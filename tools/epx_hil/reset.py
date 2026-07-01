"""Reboot the device and validate recovery via the firmware's ``#boot`` event.

Two methods:

* ``r`` (default) — the firmware console command. The J-Link CDC UART is provided
  by the on-board debugger, not the nRF, so the COM port *survives* the nRF reset
  and we keep the same link. The firmware emits ``#reboot`` (pre-reset) and
  ``#boot`` (on restart); we confirm recovery by matching rot/gear.
* ``nrfjprog`` — external ``nrfjprog -f nrf52 --reset``. This drops/re-enumerates
  the CDC port, so we close the link, reset, re-autodetect and reopen.

The ``#boot`` mask is on by default (EVT_LOG_DEFAULT_MASK 0x3F), so no setup is
needed for the event to appear.
"""

from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass

from . import commands, probe, protocol
from .link import SerialLink, TranscriptCb
from .models import Event
from .units import turns_at_cd


@dataclass
class RebootResult:
    method: str
    boot_event: Event | None
    base_gear: int | None
    base_rot: int | None
    boot_gear: int | None
    boot_rot: int | None
    recovered: bool
    message: str

    def to_dict(self) -> dict:
        d = dict(self.__dict__)
        d["boot_event"] = self.boot_event.raw if self.boot_event else None
        return d


def _sample_rotations(link: SerialLink) -> int | None:
    """Grab one telemetry row and derive the absolute turn count."""
    link.start_tele_capture()
    link.send_nowait(protocol.cmd_telemetry(4))
    time.sleep(0.2)
    rows = link.stop_tele_capture()
    link.send_nowait(protocol.cmd_telemetry(0))
    return turns_at_cd(rows[-1].current_cd) if rows else None


def _baseline(link: SerialLink):
    st = commands.read_status(link)
    return (st.gear if st else None), _sample_rotations(link)


def wait_for_boot(link: SerialLink, since_ts: float, timeout: float) -> Event | None:
    """Poll the link's event stream for a ``#boot`` at/after ``since_ts``."""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        for ev in link.events_since(since_ts):
            if ev.kind == "boot":
                return ev
        time.sleep(0.05)
    return None


def _make_result(method, base_gear, base_rot, boot: Event | None) -> RebootResult:
    boot_gear = boot.fields.get("gear") if boot else None
    boot_rot = boot.fields.get("rot") if boot else None
    if boot is None:
        return RebootResult(method, None, base_gear, base_rot, None, None, False,
                            "no #boot event seen after reboot")
    recovered = (
        (base_gear is None or boot_gear == base_gear)
        and (base_rot is None or boot_rot == base_rot)
    )
    msg = "recovered" if recovered else (
        f"recovery mismatch: pre gear={base_gear} rot={base_rot}, "
        f"boot gear={boot_gear} rot={boot_rot}")
    return RebootResult(method, boot, base_gear, base_rot, boot_gear, boot_rot, recovered, msg)


def reboot(
    link: SerialLink,
    method: str = "r",
    *,
    baud: int = 115200,
    transcript_cb: TranscriptCb | None = None,
    timeout: float = 10.0,
    log=print,
) -> tuple[SerialLink, RebootResult]:
    """Reboot the device; returns ``(link, RebootResult)`` (link may be new for nrfjprog)."""
    base_gear, base_rot = _baseline(link)
    log(f"reboot({method}): baseline gear={base_gear} rot={base_rot}")

    if method == "r":
        since = time.perf_counter()
        link.send_and_await(protocol.cmd_reboot(),
                            expect=lambda ln: ln.startswith("Rebooting"), timeout=1.0)
        boot = wait_for_boot(link, since, timeout)
        return link, _make_result("r", base_gear, base_rot, boot)

    if method == "nrfjprog":
        port = link.port
        link.close()
        log("reboot(nrfjprog): nrfjprog -f nrf52 --reset")
        subprocess.run(["nrfjprog", "-f", "nrf52", "--reset"],
                       check=True, capture_output=True, text=True)
        time.sleep(2.0)  # CDC re-enumeration
        newport = probe.autodetect(baud)
        log(f"reboot(nrfjprog): reconnected on {newport}")
        newlink = SerialLink(newport, baud, transcript=transcript_cb).open()
        boot = wait_for_boot(newlink, 0.0, timeout)
        return newlink, _make_result("nrfjprog", base_gear, base_rot, boot)

    raise ValueError(f"unknown reboot method {method!r} (use 'r' or 'nrfjprog')")
