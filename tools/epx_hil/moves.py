"""Logged-move orchestration: one gear/angle move -> captured window -> metrics -> log."""

from __future__ import annotations

import time

from . import commands, metrics, protocol
from .link import SerialLink
from .logging_ import LogSession
from .models import GearTable, MoveMetrics
from .telemetry import capture_move, wait_settle
from .units import deg_to_cd

# Small pre-roll so a few pre-move telemetry rows land before the command, making
# the target-change edge (metrics t0) unambiguous.
_PREROLL_S = 0.10


def run_gear_move(
    link: SerialLink,
    session: LogSession | None,
    gear_table: GearTable,
    to_index: int,
    *,
    label: str | None = None,
    divider: int = 2,
    settle_timeout: float = 12.0,
    band_deg: float = 10.0,
) -> MoveMetrics:
    """Shift to a 0-based gear index, capturing + scoring the move."""
    commands.ensure_gear_mode(link)
    start = commands.read_status(link)
    from_g = start.logical_gear if start else "?"
    label = label or f"g{from_g}->g{to_index + 1}"
    final_cd = deg_to_cd(gear_table.position_for_index(to_index))

    with capture_move(link, divider=divider, tag=label) as win:
        time.sleep(_PREROLL_S)
        win.t_cmd = link.send_nowait(protocol.cmd_shift_to_index(to_index))
        res = wait_settle(link, timeout=settle_timeout)
        win.settled, win.fault = res.ok, res.fault

    m = metrics.compute(label, win.rows, final_cd, band_deg=band_deg,
                        turn_events=win.turn_events(), fault=win.fault, settled=win.settled)
    if session:
        session.add_move(label, win.rows, m, params={
            "to_index": to_index, "final_target_deg": gear_table.position_for_index(to_index)})
    return m


def run_angle_move(
    link: SerialLink,
    session: LogSession | None,
    to_deg: float,
    *,
    label: str | None = None,
    divider: int = 2,
    settle_timeout: float = 12.0,
    band_deg: float = 10.0,
) -> MoveMetrics:
    """Move to an absolute angle (angle mode), capturing + scoring the move."""
    commands.ensure_angle_mode(link)
    start = commands.read_status(link)
    label = label or f"angle->{to_deg:.0f}"
    final_cd = deg_to_cd(to_deg)

    with capture_move(link, divider=divider, tag=label) as win:
        time.sleep(_PREROLL_S)
        win.t_cmd = link.send_nowait(protocol.cmd_set_angle(to_deg))
        res = wait_settle(link, timeout=settle_timeout)
        win.settled, win.fault = res.ok, res.fault

    m = metrics.compute(label, win.rows, final_cd, band_deg=band_deg,
                        turn_events=win.turn_events(), fault=win.fault, settled=win.settled)
    if session:
        session.add_move(label, win.rows, m, params={"to_deg": to_deg})
    return m
