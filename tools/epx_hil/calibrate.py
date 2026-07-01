"""Automated two-point calibration (gear 2 + gear 10) driven over the console.

The firmware's console cal path is: enter angle mode, jog with ``t<abs>``, capture
the low ref with ``gl``, jog again, capture the high ref with ``gh``, then ``gi``
to affine-fit the nominal cog profile through the two refs and persist. Nothing
auto-sets ``current_gear`` afterward, so we finish by entering gear mode and
``s9`` (logical gear 10) to seat the device in gear 10.

Gear 2 and gear 10 are captured at two **distinct, symmetric** angles — each
``span/2`` away from the entry position (gear 2 the higher angle, gear 10 the
lower, per the EPS profile), i.e. "similar distances away".

Jogging is robust to the current control reality: with Kp-only gains the motor
holds with a standing error larger than the firmware's settle band, so it reports
``MOV`` forever and never reaches ``HLD``. We therefore jog by waiting for the
**position to stop changing** (not for a HLD that never comes), accept "stopped
near the target" as arrival (the standing error keeps it from landing exactly),
and only treat no-motion *far* from the target as a real stall/end-stop — which
aborts to a safe hold (protects a real derailleur's hard stops).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from . import commands, protocol
from .link import SerialLink
from .models import GEAR_IDX_HIGH, CalibrationResult, GearTable


class CalibrationError(RuntimeError):
    pass


@dataclass
class CalibrationConfig:
    span_deg: float = 5392.0             # gear2..gear10 total travel (EPS profile)
    g2_angle_deg: float | None = None    # absolute overrides (else entry ± span/2)
    g10_angle_deg: float | None = None
    # Jog behaviour / safety.
    jog_step_deg: float = 90.0
    jog_tol_deg: float = 3.0
    jog_step_timeout: float = 3.0        # per-step: wait for motion to stop
    arrival_band_deg: float = 90.0       # "stopped this close to target" == arrived
    stall_eps_deg: float = 2.0           # per-step motion below this == no progress
    max_stalls: int = 3                  # consecutive no-progress steps far from target
    motion_stable_eps_deg: float = 3.0   # |Δpos| under this == stable
    motion_stable_count: int = 4         # consecutive stable samples == stopped
    poll_s: float = 0.08
    max_total_jog_deg: float = 0.0       # 0 => derive from span
    settle_timeout: float = 12.0
    # Span sanity (gear2..gear10).
    span_min_deg: float = 100.0
    span_max_deg: float = 7000.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class Calibrator:
    def __init__(self, link: SerialLink, config: CalibrationConfig | None = None, log=print):
        self.link = link
        self.cfg = config or CalibrationConfig()
        self.log = log

    def plan(self) -> dict:
        """Compute the (symmetric) jog plan without moving (``--dry-jog``)."""
        st = commands.read_status(self.link)
        if st is None:
            raise CalibrationError("no status reply; is the device connected?")
        entry = st.pos_deg
        half = self.cfg.span_deg / 2.0
        g2 = self.cfg.g2_angle_deg if self.cfg.g2_angle_deg is not None else entry + half
        g10 = self.cfg.g10_angle_deg if self.cfg.g10_angle_deg is not None else entry - half
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

        # Seat gear 10 at the current (high-ref) position without a big move.
        commands.ensure_gear_mode(self.link)
        self.link.send_nowait(protocol.cmd_shift_to_index(GEAR_IDX_HIGH))
        self._wait_motion_stop(self.cfg.settle_timeout)
        st = commands.read_status(self.link)
        if st is None or st.gear != GEAR_IDX_HIGH:
            flags.append("post_cal_gear_mismatch")
        if st is not None and st.fault:
            flags.append("post_cal_fault")

        ok = not flags
        return CalibrationResult(low_ref_deg=low, high_ref_deg=high, span_deg=span,
                                 gear_table=table, ok=ok, flags=flags)

    # ----- internals ------------------------------------------------------- #
    def _jog_cap(self) -> float:
        return self.cfg.max_total_jog_deg or (self.cfg.span_deg * 1.5 + 500.0)

    def _safe_hold(self) -> None:
        """Command the motor to hold the current sensor position (stop driving)."""
        st = commands.read_status(self.link)
        if st:
            self.link.send_nowait(protocol.cmd_set_angle(st.pos_deg))

    def _wait_motion_stop(self, timeout: float):
        """Poll ``?`` until the position stops changing (or fault/timeout).

        Returns ``(status, faulted)``. Independent of MOV/HLD, so it works even
        when the standing error keeps the firmware reporting MOV forever.
        """
        deadline = time.perf_counter() + timeout
        last = None
        stable = 0
        st = None
        while time.perf_counter() < deadline:
            st = commands.read_status(self.link)
            if st is None:
                continue
            if st.fault:
                return st, True
            if last is not None and abs(st.pos_deg - last) <= self.cfg.motion_stable_eps_deg:
                stable += 1
                if stable >= self.cfg.motion_stable_count:
                    return st, False
            else:
                stable = 0
            last = st.pos_deg
            time.sleep(self.cfg.poll_s)
        return st, False

    def _jog_to(self, target_abs: float) -> None:
        st = commands.read_status(self.link)
        if st is None:
            raise CalibrationError("no status during jog")
        cap = self._jog_cap()
        if abs(target_abs - st.pos_deg) > cap:
            raise CalibrationError(
                f"jog to {target_abs:.0f}deg from {st.pos_deg}deg exceeds cap {cap:.0f}deg")

        max_iters = int(abs(target_abs - st.pos_deg) / max(1.0, self.cfg.jog_step_deg)) * 3 + 60
        stalls = 0
        for _ in range(max_iters):
            st = commands.read_status(self.link)
            if st is None:
                continue                      # dropped read: retry, not a stall
            if st.fault:
                self._safe_hold()
                raise CalibrationError("fault during jog")
            remaining = target_abs - st.pos_deg
            if abs(remaining) <= self.cfg.jog_tol_deg:
                return
            before = st.pos_deg
            step = _clamp(remaining, -self.cfg.jog_step_deg, self.cfg.jog_step_deg)
            self.link.send_nowait(protocol.cmd_set_angle(before + step))
            after, faulted = self._wait_motion_stop(self.cfg.jog_step_timeout)
            if faulted:
                self._safe_hold()
                raise CalibrationError("fault while jogging")
            pos_after = after.pos_deg if after else before
            if abs(pos_after - before) < self.cfg.stall_eps_deg:
                # No progress. Near the target this is just the standing error /
                # tolerance -> accept as arrived. Far from target it's real
                # resistance (an end stop) -> abort to a safe hold.
                if abs(target_abs - pos_after) <= self.cfg.arrival_band_deg:
                    return
                stalls += 1
                if stalls >= self.cfg.max_stalls:
                    self._safe_hold()
                    raise CalibrationError(
                        f"no motion at {pos_after}deg, {abs(target_abs - pos_after):.0f}deg "
                        f"from target (resistance / end stop?)")
            else:
                stalls = 0
        self._safe_hold()
        raise CalibrationError(f"jog to {target_abs:.0f}deg did not converge")

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
