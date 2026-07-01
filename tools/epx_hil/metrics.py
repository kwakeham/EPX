"""Per-move metrics over a captured telemetry window. Pure — no I/O.

All math is in centi-degrees (telemetry native); only the reported ``MoveMetrics``
is converted to degrees/percent/ms. The **final target** is supplied by the
caller (from the parsed gear table for shifts, or the requested angle for angle
moves) and is NOT read from the momentary ``target`` column, because a long shift
steps that column through an overshift *waypoint* before the final gear position.
Overshoot is therefore only counted *after* the trajectory first reaches the final
band, so the commanded overshift travel is not mistaken for error.
"""

from __future__ import annotations

import math

from .models import MoveMetrics, TelemetryRow
from .units import cd_to_deg, deg_to_cd, turns_at_cd


def _sign(x: float) -> int:
    return (x > 0) - (x < 0)


def compute(
    label: str,
    rows: list[TelemetryRow],
    final_target_cd: int,
    *,
    band_deg: float = 10.0,
    motion_eps_deg: float = 0.5,
    settle_tail: int = 32,
    turn_events: int = 0,
    fault: bool = False,
    settled: bool = True,
) -> MoveMetrics:
    """Compute :class:`MoveMetrics` for one move.

    ``band_deg`` mirrors the firmware's ±10° settle band. ``turn_events`` is the
    number of ``#turn`` events counted in the window (ground truth for the turn
    count). ``settled``/``fault`` come from the capture (settle reached / fault
    aborted); metrics on a partial window are still computed and flagged.
    """
    final_deg = cd_to_deg(final_target_cd)

    if not rows:
        return MoveMetrics(
            label=label, final_target_deg=final_deg, start_deg=math.nan,
            distance_deg=math.nan, time_to_move_ms=None, time_to_target_ms=None,
            settle_time_fw_ms=None, settle_time_band_ms=None, overshoot_deg=None,
            overshoot_pct=None, steady_state_err_deg=None, peak_isense=None,
            hold_isense=None, turns_traversed=0, turn_events=turn_events,
            n_rows=0, flags=["no_rows"],
        )

    band_cd = deg_to_cd(band_deg)
    motion_eps_cd = deg_to_cd(motion_eps_deg)
    flags: list[str] = []

    # --- move start (t0): first row whose target left the initial value -------
    init_target = rows[0].target_cd
    t0_idx = 0
    for i, r in enumerate(rows):
        if r.target_cd != init_target:
            t0_idx = i
            break
    t0_ms = rows[t0_idx].t_ms
    start_cd = rows[t0_idx].current_cd
    start_deg = cd_to_deg(start_cd)
    distance_cd = final_target_cd - start_cd
    distance_deg = cd_to_deg(distance_cd)
    direction = _sign(distance_cd)

    move_rows = rows[t0_idx:]

    # --- first-motion latency -------------------------------------------------
    time_to_move_ms = None
    for r in move_rows:
        if abs(r.current_cd - start_cd) >= motion_eps_cd or r.moving:
            time_to_move_ms = float(r.t_ms - t0_ms)
            break

    # --- rise: first entry into the final band --------------------------------
    time_to_target_ms = None
    firstinband_idx = None
    for j, r in enumerate(move_rows):
        if abs(r.current_cd - final_target_cd) <= band_cd:
            time_to_target_ms = float(r.t_ms - t0_ms)
            firstinband_idx = j
            break

    # --- overshoot vs FINAL target, only after first reaching the band --------
    overshoot_deg = None
    overshoot_pct = None
    if firstinband_idx is not None and direction != 0:
        peak_cd = 0
        for r in move_rows[firstinband_idx:]:
            excursion = direction * (r.current_cd - final_target_cd)
            if excursion > peak_cd:
                peak_cd = excursion
        overshoot_deg = cd_to_deg(peak_cd)
        overshoot_pct = 100.0 * peak_cd / abs(distance_cd) if distance_cd else math.nan

    # --- settle (firmware MOV->HLD edge) --------------------------------------
    settle_time_fw_ms = None
    seen_moving = False
    for r in move_rows:
        if r.moving:
            seen_moving = True
        elif seen_moving:  # first HLD after having moved
            settle_time_fw_ms = float(r.t_ms - t0_ms)
            break

    # --- settle (computed: start of the final sustained in-band run) ----------
    settle_time_band_ms = None
    if abs(move_rows[-1].current_cd - final_target_cd) <= band_cd:
        start_run = len(move_rows) - 1
        while start_run > 0 and abs(move_rows[start_run - 1].current_cd - final_target_cd) <= band_cd:
            start_run -= 1
        settle_time_band_ms = float(move_rows[start_run].t_ms - t0_ms)

    # --- steady-state error over trailing HLD rows ----------------------------
    hold_rows = [r for r in move_rows if not r.moving]
    tail = (hold_rows or move_rows)[-settle_tail:]
    steady_state_err_deg = cd_to_deg(
        sum(abs(r.current_cd - final_target_cd) for r in tail) / len(tail)
    )
    hold_isense = (sum(r.isense for r in hold_rows) / len(hold_rows)) if hold_rows else None

    # --- current + turns ------------------------------------------------------
    peak_isense = max(r.isense for r in move_rows)
    end_cd = move_rows[-1].current_cd
    turns_traversed = turns_at_cd(end_cd) - turns_at_cd(start_cd)

    # --- flags ----------------------------------------------------------------
    if fault or any(r.fault for r in move_rows):
        flags.append("fault")
    if not settled or move_rows[-1].moving:
        flags.append("settle_timeout")
    if time_to_move_ms is None and abs(distance_cd) > motion_eps_cd:
        flags.append("no_motion")
    if firstinband_idx is None:
        flags.append("never_in_band")
    if turn_events and abs(turns_traversed) != turn_events:
        flags.append("turn_count_discrepancy")
    last_target_cd = move_rows[-1].target_cd
    if abs(last_target_cd - final_target_cd) > deg_to_cd(1.0):
        flags.append("overshift_mismatch")

    return MoveMetrics(
        label=label, final_target_deg=final_deg, start_deg=start_deg,
        distance_deg=distance_deg, time_to_move_ms=time_to_move_ms,
        time_to_target_ms=time_to_target_ms, settle_time_fw_ms=settle_time_fw_ms,
        settle_time_band_ms=settle_time_band_ms, overshoot_deg=overshoot_deg,
        overshoot_pct=overshoot_pct, steady_state_err_deg=steady_state_err_deg,
        peak_isense=peak_isense, hold_isense=hold_isense,
        turns_traversed=turns_traversed, turn_events=turn_events,
        n_rows=len(move_rows), flags=flags,
    )
