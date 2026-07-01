"""Automated two-point calibration (gear 2 + gear 10) driven over the console.

The firmware's console cal path is: enter angle mode, jog with ``t<abs>``, capture
the low ref with ``gl``, jog again, capture the high ref with ``gh``, then ``gi``
to affine-fit the nominal cog profile through the two refs and persist. Nothing
auto-sets ``current_gear`` afterward, so we finish by entering gear mode and
``s9`` (logical gear 10) — whose target equals the just-captured high ref, so the
device settles in gear 10 without a large move.

Safety (full assembled derailleur, ~5400° between gear 2 and gear 10): every jog
is a bounded step computed from a freshly-read sensor angle, capped in total
travel, settled per step, and aborted on a fault or a stall (no motion => a hard
stop). We never send a blind absolute target that could slam a stop.
"""

from __future__ import annotations

from dataclasses import dataclass

from . import commands, protocol
from .link import SerialLink
from .models import GEAR_IDX_HIGH, CalibrationResult, GearTable


class CalibrationError(RuntimeError):
    pass


@dataclass
class CalibrationConfig:
    # Capture targets: absolute angle if given, else entry position + delta.
    g2_delta_deg: float = 0.0            # gear 2 captured at the entry position
    g10_delta_deg: float = -5392.0       # gear 10 ~5392 deg below gear 2 (EPS profile)
    g2_angle_deg: float | None = None
    g10_angle_deg: float | None = None
    # Jog safety envelope.
    jog_step_deg: float = 90.0
    jog_tol_deg: float = 3.0
    max_total_jog_deg: float = 6000.0
    stall_eps_deg: float = 2.0
    max_stalls: int = 2
    jog_settle_timeout: float = 6.0
    settle_timeout: float = 12.0
    # Span sanity (gear2..gear10).
    span_min_deg: float = 500.0
    span_max_deg: float = 7000.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class Calibrator:
    def __init__(self, link: SerialLink, config: CalibrationConfig | None = None, log=print):
        self.link = link
        self.cfg = config or CalibrationConfig()
        self.log = log

    def plan(self) -> dict:
        """Compute the jog plan without moving (``--dry-jog``)."""
        st = commands.read_status(self.link)
        if st is None:
            raise CalibrationError("no status reply; is the device connected?")
        entry = st.pos_deg
        g2 = self.cfg.g2_angle_deg if self.cfg.g2_angle_deg is not None else entry + self.cfg.g2_delta_deg
        g10 = self.cfg.g10_angle_deg if self.cfg.g10_angle_deg is not None else entry + self.cfg.g10_delta_deg
        return {"entry_deg": entry, "g2_target_deg": g2, "g10_target_deg": g10,
                "span_deg": abs(g10 - g2), "jog_step_deg": self.cfg.jog_step_deg}

    def run(self) -> CalibrationResult:
        flags: list[str] = []
        commands.clear_fault(self.link)
        if not commands.ensure_angle_mode(self.link):
            raise CalibrationError("could not enter angle mode")

        p = self.plan()
        self.log(f"cal: entry={p['entry_deg']}deg  g2->{p['g2_target_deg']:.0f}deg  "
                 f"g10->{p['g10_target_deg']:.0f}deg  span={p['span_deg']:.0f}deg")

        self._jog_to(p["g2_target_deg"])
        low = self._capture(protocol.cmd_capture_low(), protocol.parse_low_ref, "gear 2")
        self.log(f"cal: captured low ref (gear 2) = {low}")

        self._jog_to(p["g10_target_deg"])
        high = self._capture(protocol.cmd_capture_high(), protocol.parse_high_ref, "gear 10")
        self.log(f"cal: captured high ref (gear 10) = {high}")

        table = self._interpolate()
        span = abs(high - low)
        self.log(f"cal: fit span={span}deg, table={table.positions_deg if table else None}")

        if not (self.cfg.span_min_deg <= span <= self.cfg.span_max_deg):
            flags.append("span_out_of_range")
        if table is None or not table.is_calibrated():
            flags.append("gear_table_invalid")

        # Establish gear 10 at the current (high-ref) position without a big move.
        commands.ensure_gear_mode(self.link)
        self.link.send_nowait(protocol.cmd_shift_to_index(GEAR_IDX_HIGH))
        commands.wait_settle_status(self.link, timeout=self.cfg.settle_timeout)
        st = commands.read_status(self.link)
        if st is None or st.gear != GEAR_IDX_HIGH:
            flags.append("post_cal_gear_mismatch")
        if st is not None and st.moving:
            flags.append("post_cal_not_settled")
        if st is not None and st.fault:
            flags.append("post_cal_fault")

        ok = not flags
        return CalibrationResult(low_ref_deg=low, high_ref_deg=high, span_deg=span,
                                 gear_table=table, ok=ok, flags=flags)

    # ----- internals ------------------------------------------------------- #
    def _jog_to(self, target_abs: float) -> None:
        st = commands.read_status(self.link)
        if st is None:
            raise CalibrationError("no status during jog")
        if abs(target_abs - st.pos_deg) > self.cfg.max_total_jog_deg:
            raise CalibrationError(
                f"jog to {target_abs:.0f}deg from {st.pos_deg}deg exceeds safety cap "
                f"{self.cfg.max_total_jog_deg:.0f}deg")
        stalls = 0
        while True:
            st = commands.read_status(self.link)
            if st is None:
                raise CalibrationError("no status during jog")
            if st.fault:
                raise CalibrationError("fault during jog")
            remaining = target_abs - st.pos_deg
            if abs(remaining) <= self.cfg.jog_tol_deg:
                return
            step = _clamp(remaining, -self.cfg.jog_step_deg, self.cfg.jog_step_deg)
            before = st.pos_deg
            self.link.send_nowait(protocol.cmd_set_angle(before + step))
            res = commands.wait_settle_status(self.link, timeout=self.cfg.jog_settle_timeout)
            if res.fault:
                raise CalibrationError("fault while settling a jog step")
            after = commands.read_status(self.link)
            moved = abs((after.pos_deg if after else before) - before)
            if moved < self.cfg.stall_eps_deg:
                stalls += 1
                if stalls >= self.cfg.max_stalls:
                    raise CalibrationError(
                        f"no motion near {before}deg (stall / hard stop) after {stalls} steps")
            else:
                stalls = 0

    def _capture(self, cmd: str, parser, label: str) -> int:
        r = self.link.send_and_await(cmd, expect=lambda ln: parser(ln) is not None, timeout=1.5)
        for ln in r.lines:
            v = parser(ln)
            if v is not None:
                return v
        raise CalibrationError(f"no capture reply for {label} (sent {cmd!r})")

    def _interpolate(self) -> GearTable | None:
        def have_both(lines: list[str]) -> bool:
            firsts = {protocol.parse_gear_table(ln)[0]
                      for ln in lines if protocol.parse_gear_table(ln)}
            return {1, 7}.issubset(firsts)

        r = self.link.send_and_await(protocol.cmd_interpolate(), until=have_both, timeout=2.5)
        tbl_lines = [ln for ln in r.lines if protocol.parse_gear_table(ln)]
        return protocol.gear_table_from_lines(tbl_lines)
