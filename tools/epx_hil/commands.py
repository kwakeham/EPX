"""High-level console operations built on :class:`~epx_hil.link.SerialLink`.

Thin wrappers that send a command and parse its reply into a typed record. Used
by the move runner, the calibration FSM and the test battery so the exact
command/parse pairing lives in one place.
"""

from __future__ import annotations

import time

from . import protocol
from .link import SerialLink
from .models import GearTable, Status
from .telemetry import SettleResult


def read_status(link: SerialLink, timeout: float = 1.0) -> Status | None:
    r = link.send_and_await(
        protocol.cmd_status(),
        expect=lambda ln: protocol.parse_status(ln) is not None,
        timeout=timeout,
    )
    for ln in r.lines:
        st = protocol.parse_status(ln)
        if st:
            return st
    return None


def read_gear_table(link: SerialLink, timeout: float = 1.5) -> GearTable | None:
    def have_both(lines: list[str]) -> bool:
        got = [ln for ln in lines if protocol.parse_gear_table(ln)]
        firsts = {protocol.parse_gear_table(ln)[0] for ln in got}
        return {1, 7}.issubset(firsts)

    r = link.send_and_await(protocol.cmd_gear_table(), until=have_both, timeout=timeout)
    tbl_lines = [ln for ln in r.lines if protocol.parse_gear_table(ln)]
    return protocol.gear_table_from_lines(tbl_lines)


def read_gains(link: SerialLink, timeout: float = 1.0):
    r = link.send_and_await(
        protocol.cmd_list_gains(),
        expect=lambda ln: protocol.parse_gains(ln) is not None,
        timeout=timeout,
    )
    for ln in r.lines:
        g = protocol.parse_gains(ln)
        if g:
            return g
    return None


def ensure_gear_mode(link: SerialLink, timeout: float = 1.0) -> bool:
    r = link.send_and_await(protocol.cmd_mode_gear(),
                            expect=lambda ln: ln == "Gear Mode", timeout=timeout)
    return r.matched


def ensure_angle_mode(link: SerialLink, timeout: float = 1.0) -> bool:
    r = link.send_and_await(protocol.cmd_mode_angle(),
                            expect=lambda ln: ln == "Angle Mode", timeout=timeout)
    return r.matched


def clear_fault(link: SerialLink, timeout: float = 1.0) -> bool:
    r = link.send_and_await(protocol.cmd_clear_fault(),
                            expect=lambda ln: ln == "Fault cleared", timeout=timeout)
    return r.matched


def set_isense_limit(link: SerialLink, counts: int, timeout: float = 1.0) -> bool:
    r = link.send_and_await(protocol.cmd_isense_limit(counts),
                            expect=lambda ln: ln.startswith("ISENSE limit"), timeout=timeout)
    return r.matched


def set_isense_count(link: SerialLink, n: int, timeout: float = 1.0) -> bool:
    r = link.send_and_await(protocol.cmd_isense_count(n),
                            expect=lambda ln: ln.startswith("ISENSE count"), timeout=timeout)
    return r.matched


def wait_settle_status(
    link: SerialLink,
    timeout: float = 8.0,
    noop_grace: float = 0.6,
    poll: float = 0.1,
) -> SettleResult:
    """Settle detection by polling ``?`` (for use when telemetry is off, e.g. cal jogs).

    Mirrors :func:`telemetry.wait_settle`: returns once the device reaches HLD
    after moving, immediately on a fault, or after ``noop_grace`` if it never left
    HLD (a no-op move).
    """
    start = time.perf_counter()
    grace = start + noop_grace
    seen_moving = False
    while time.perf_counter() < start + timeout:
        st = read_status(link, timeout=poll + 0.4)
        if st is not None:
            if st.fault:
                return SettleResult(False, True, "fault", None)
            if st.moving:
                seen_moving = True
            elif seen_moving:
                return SettleResult(True, False, "settled", None)
            elif time.perf_counter() > grace:
                return SettleResult(True, False, "noop", None)
        time.sleep(poll)
    return SettleResult(False, False, "timeout", None)
