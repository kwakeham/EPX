"""Per-move telemetry capture + settle detection.

``capture_move`` is a context manager: it enables the ``#event`` mask and CSV
telemetry, collects every row/event for the window, and on exit turns telemetry
off and hands back the captured rows + events. Inside the window the caller
issues the move command and calls :func:`wait_settle`, which watches the live
``state`` column for the firmware's own MOV->HLD move-complete edge (and aborts
early on a fault).
"""

from __future__ import annotations

import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field

from . import protocol
from .link import SerialLink
from .models import Event, TelemetryRow

EVENT_MASK_ALL = 0x3F


@dataclass
class MoveWindow:
    """Everything captured for one move."""

    tag: str
    divider: int
    t_open: float
    t_cmd: float | None = None
    rows: list[TelemetryRow] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)
    settled: bool = True
    fault: bool = False

    def turn_events(self) -> int:
        return sum(1 for e in self.events if e.kind == "turn")


@dataclass
class SettleResult:
    ok: bool
    fault: bool
    reason: str
    last: TelemetryRow | None


@contextmanager
def capture_move(link: SerialLink, divider: int = 2, event_mask: int = EVENT_MASK_ALL, tag: str = "move"):
    """Enable telemetry+events, capture the window, disable on exit."""
    link.start_tele_capture()
    link.send_nowait(protocol.cmd_events(event_mask))
    link.send_nowait(protocol.cmd_telemetry(divider))
    win = MoveWindow(tag=tag, divider=divider, t_open=time.perf_counter())
    try:
        yield win
    finally:
        link.send_nowait(protocol.cmd_telemetry(0))
        time.sleep(0.06)  # let the last few rows drain before we snapshot
        win.rows = link.stop_tele_capture()
        win.events = link.events_since(win.t_open)


def wait_settle(
    link: SerialLink,
    timeout: float = 8.0,
    noop_grace: float = 0.6,
) -> SettleResult:
    """Block until the move settles (MOV->HLD), a fault latches, or timeout.

    Handles the no-op case (a move that never leaves HLD because it was already
    at target): if no MOV is seen within ``noop_grace`` and the device is HLD,
    it is treated as settled.
    """
    lock = threading.Lock()
    st = {"fault": False, "settled": False, "seen_moving": False, "last": None}

    def on_row(r: TelemetryRow) -> None:
        with lock:
            st["last"] = r
            if r.fault:
                st["fault"] = True
            if r.moving:
                st["seen_moving"] = True
            elif st["seen_moving"]:
                st["settled"] = True

    link.set_row_callback(on_row)
    try:
        deadline = time.perf_counter() + timeout
        grace = time.perf_counter() + noop_grace
        while time.perf_counter() < deadline:
            with lock:
                last = st["last"]
                if st["fault"]:
                    return SettleResult(False, True, "fault", last)
                if st["settled"]:
                    return SettleResult(True, False, "settled", last)
                noop = (not st["seen_moving"] and last is not None
                        and not last.moving and time.perf_counter() > grace)
            if noop:
                return SettleResult(True, False, "noop", last)
            time.sleep(0.01)
        with lock:
            return SettleResult(False, st["fault"], "timeout", st["last"])
    finally:
        link.set_row_callback(None)
