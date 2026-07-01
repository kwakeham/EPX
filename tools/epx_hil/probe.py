"""COM-port autodetection.

The J-Link CDC COM number churns across re-enumeration (COM5/7/9/10 seen on this
bench) and there are often two CDC ports, so we probe rather than hard-code:
rank SEGGER/J-Link ports first, open each at 115200, send ``?`` and accept the
one that answers with a parseable status line. Probing sends only read-only
commands (``?``), never anything that moves the motor.
"""

from __future__ import annotations

import serial.tools.list_ports

from . import protocol
from .link import SerialLink

SEGGER_VID = 0x1366


class NoDeviceError(RuntimeError):
    pass


def list_candidate_ports():
    """Return ``list[ListPortInfo]`` ranked with likely EPX (J-Link CDC) ports first."""
    ports = list(serial.tools.list_ports.comports())

    def rank(p) -> int:
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if p.vid == SEGGER_VID:
            return 0
        if "jlink" in desc.replace(" ", "") or "segger" in desc or "cdc uart" in desc:
            return 1
        return 2

    return sorted(ports, key=rank)


def probe_port(device: str, baud: int = 115200, timeout: float = 0.8) -> bool:
    """True if a device on ``device`` answers ``?`` with a parseable status line."""
    link = SerialLink(device, baud)
    try:
        link.open()
    except (serial.SerialException, OSError):
        return False
    try:
        r = link.send_and_await(
            protocol.cmd_status(),
            expect=lambda ln: protocol.parse_status(ln) is not None,
            timeout=timeout,
        )
        return r.matched
    finally:
        link.close()


def autodetect(baud: int = 115200, preferred: str | None = None) -> str:
    """Return the COM device that answers as EPX, or raise :class:`NoDeviceError`."""
    if preferred:
        if probe_port(preferred, baud):
            return preferred
        raise NoDeviceError(f"{preferred} did not answer as an EPX device")

    tried = []
    for p in list_candidate_ports():
        tried.append(p.device)
        if probe_port(p.device, baud):
            return p.device
    raise NoDeviceError(
        "no EPX device answered '?' on any COM port. Tried: " + (", ".join(tried) or "(none)")
    )
