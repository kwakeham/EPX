"""EPX console grammar: command builders + line classification/parsers.

Pure functions only (no I/O), so this is the unit-tested core. Every command
string is the *exact* form the firmware's dispatcher parses:

* Handlers that read a fixed character index (``command_message[1]``) take **no
  space**: mode is ``ma``/``mg``, gear cal is ``gl``/``gh``/``gi`` (a space would
  land ' ' in that slot and miss the case). Confirmed in
  ``data_handler_command_gear_value`` / ``data_handler_shift_mode_handler``.
* Handlers that parse with ``atoi``/``sscanf`` tolerate a leading space, so
  ``xl 5``, ``xc 3`` and the 5-field ``o`` command are written with spaces.

Reply/telemetry/event line shapes are matched verbatim against the firmware's
``dh_reply``/``telemetry.c``/``evt_log`` format strings.
"""

from __future__ import annotations

import re

from .models import Event, GearTable, LineTag, Status

# --------------------------------------------------------------------------- #
# Command builders
# --------------------------------------------------------------------------- #

CSV_HEADER = "t_ms,target,current,error,drive,integral,state,isense,fault"
BANNER_PREFIX = "EPX console ready"


def cmd_status() -> str:
    return "?"


def cmd_help() -> str:
    return "h"


def cmd_mode_angle() -> str:
    return "ma"


def cmd_mode_gear() -> str:
    return "mg"


def cmd_set_angle(deg: float) -> str:
    return f"t{deg:.2f}"


def cmd_shift_to_index(gear_index: int) -> str:
    """Direct shift to a 0-based internal gear index (gear mode only)."""
    return f"s{int(gear_index)}"


def cmd_shift_up() -> str:
    return "s+"


def cmd_shift_down() -> str:
    return "s-"


def cmd_calibrate_toggle() -> str:
    return "c"


def cmd_capture_low() -> str:
    return "gl"


def cmd_capture_high() -> str:
    return "gh"


def cmd_interpolate() -> str:
    return "gi"


def cmd_gear_table() -> str:
    return "g"


def cmd_set_gear_count(n: int) -> str:
    return f"gf {int(n)}"


def cmd_telemetry(divider: int) -> str:
    return f"y{int(divider)}"


def cmd_monitor(divider: int) -> str:
    return f"v{int(divider)}"


def cmd_events(mask: int) -> str:
    return f"e{int(mask)}"


def cmd_events_query() -> str:
    return "e"


def cmd_clear_fault() -> str:
    return "x"


def cmd_isense_limit(counts: int) -> str:
    return f"xl {int(counts)}"


def cmd_isense_count(n: int) -> str:
    return f"xc {int(n)}"


def cmd_set_kp(v: float) -> str:
    return f"p{v:.3f}"


def cmd_set_ki(v: float) -> str:
    return f"i{v:.3f}"


def cmd_set_kd(v: float) -> str:
    return f"d{v:.3f}"


def cmd_list_gains() -> str:
    return "k"


def cmd_force_save() -> str:
    return "fs"


def cmd_front(pos: int) -> str:
    return f"b{int(pos)}"


def cmd_reboot() -> str:
    return "r"


def cmd_overshift(gear: int, front: int, direction: int, pm: int, dwell_ms: int) -> str:
    """Set an overshift table entry (gear is 1-based per the firmware)."""
    return f"o {int(gear)} {int(front)} {int(direction)} {int(pm)} {int(dwell_ms)}"


# --------------------------------------------------------------------------- #
# Line classification + parsers
# --------------------------------------------------------------------------- #

_RE_CSV_ROW = re.compile(r"^\d+(?:,-?\d+){8}$")
_RE_STATUS = re.compile(
    r"^(?P<mode>CAL|GEAR|ANGLE) g(?P<gear>-?\d+) "
    r"pos (?P<pos>-?\d+) tgt (?P<tgt>-?\d+) err (?P<err>-?\d+) "
    r"I (?P<isense>-?\d+) (?P<state>MOV|HLD)(?P<fault> FAULT)?$"
)
_RE_GEAR_TABLE = re.compile(r"^Gear (?P<first>\d+): (?P<vals>-?\d+(?:, *-?\d+)*)$")
_RE_LOW_REF = re.compile(r"^Low ref \(gear \d+\): (?P<val>-?\d+)$")
_RE_HIGH_REF = re.compile(r"^High ref \(gear \d+\): (?P<val>-?\d+)$")
_RE_GAINS = re.compile(
    r"^Gains: Kp: (?P<kp>-?[\d.]+), Ki: (?P<ki>-?[\d.]+), Kd: (?P<kd>-?[\d.]+)$"
)


def classify_line(line: str) -> LineTag:
    """Tag one already-stripped console line by shape (cheapest/most-specific first)."""
    if not line:
        return LineTag.UNKNOWN
    if line[0] == "#":
        return LineTag.EVENT
    if line == CSV_HEADER:
        return LineTag.CSV_HEADER
    if _RE_CSV_ROW.match(line):
        return LineTag.CSV_ROW
    if line.startswith(BANNER_PREFIX):
        return LineTag.BANNER
    if _RE_STATUS.match(line):
        return LineTag.STATUS
    return LineTag.REPLY


def parse_status(line: str) -> Status | None:
    m = _RE_STATUS.match(line)
    if not m:
        return None
    return Status(
        mode=m["mode"],
        gear=int(m["gear"]),
        pos_deg=int(m["pos"]),
        tgt_deg=int(m["tgt"]),
        err_deg=int(m["err"]),
        isense=int(m["isense"]),
        moving=(m["state"] == "MOV"),
        fault=bool(m["fault"]),
    )


def parse_csv_row(line: str):
    """Return a 9-int tuple ``(t_ms, target, current, error, drive, integral, state,
    isense, fault)`` or ``None`` if the row is malformed/truncated."""
    if not _RE_CSV_ROW.match(line):
        return None
    parts = line.split(",")
    try:
        return tuple(int(p) for p in parts)
    except ValueError:
        return None


def parse_gear_table(line: str):
    """Parse one ``Gear N: a, b, c, ...`` line -> ``(first_gear, [values])`` or None."""
    m = _RE_GEAR_TABLE.match(line)
    if not m:
        return None
    first = int(m["first"])
    vals = [int(x) for x in m["vals"].split(",")]
    return first, vals


def gear_table_from_lines(lines: list[str]) -> GearTable | None:
    """Assemble a :class:`GearTable` from the two ``Gear 1:`` / ``Gear 7:`` lines."""
    slots: dict[int, int] = {}
    for ln in lines:
        parsed = parse_gear_table(ln)
        if not parsed:
            continue
        first, vals = parsed
        for i, v in enumerate(vals):
            slots[first - 1 + i] = v          # 'Gear 1:' -> index 0
    if not slots:
        return None
    n = max(slots) + 1
    positions = tuple(slots.get(i, 0) for i in range(n))
    return GearTable(positions_deg=positions)


def parse_low_ref(line: str) -> int | None:
    m = _RE_LOW_REF.match(line)
    return int(m["val"]) if m else None


def parse_high_ref(line: str) -> int | None:
    m = _RE_HIGH_REF.match(line)
    return int(m["val"]) if m else None


def parse_gains(line: str):
    """Parse ``Gains: Kp: .., Ki: .., Kd: ..`` -> ``(kp, ki, kd)`` or None."""
    m = _RE_GAINS.match(line)
    if not m:
        return None
    return float(m["kp"]), float(m["ki"]), float(m["kd"])


def _coerce(value: str):
    """Best-effort int, else str (event field values)."""
    try:
        return int(value)
    except ValueError:
        return value


def parse_event(line: str, host_ts: float = 0.0) -> Event | None:
    """Parse a ``#kind,key=value,...`` HIL event line."""
    if not line or line[0] != "#":
        return None
    body = line[1:]
    head, _, tail = body.partition(",")
    kind = head.strip()
    fields: dict = {}
    if tail:
        for tok in tail.split(","):
            if "=" in tok:
                k, _, v = tok.partition("=")
                fields[k.strip()] = _coerce(v.strip())
            elif tok.strip():
                fields[tok.strip()] = True
    return Event(kind=kind, fields=fields, host_ts=host_ts, raw=line)
