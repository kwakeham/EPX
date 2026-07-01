"""Typed records the harness parses off the wire and computes.

Field names carry their unit (``*_cd`` centi-degrees, ``*_deg`` whole degrees) so
the telemetry vs status scale split (see :mod:`units`) is visible at every call
site. All records are plain dataclasses so they serialise cleanly to JSON.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field, asdict
from typing import Any


NUM_REAR_GEARS = 11          # logical gears 1..11 == internal index 0..10
GEAR_IDX_LOW = 1             # GEAR_REF_LO_IDX  -> logical gear 2
GEAR_IDX_HIGH = 9            # GEAR_REF_HI_IDX  -> logical gear 10


class LineTag(enum.Enum):
    """Classification of one console line (the demux routes on this)."""

    EVENT = "event"           # '#tag,...' HIL event
    CSV_HEADER = "csv_header"  # 't_ms,target,current,...'
    CSV_ROW = "csv_row"       # a telemetry row
    STATUS = "status"         # '?' one-shot / verbose monitor line
    BANNER = "banner"         # startup banner
    REPLY = "reply"           # any other command reply
    UNKNOWN = "unknown"       # undecodable / malformed


@dataclass(frozen=True)
class Status:
    """Parsed ``?`` line: ``<MODE> g<idx> pos <deg> tgt <deg> err <deg> I <n> <MOV|HLD>[ FAULT]``."""

    mode: str                 # CAL | GEAR | ANGLE
    gear: int                 # 0-based internal gear index
    pos_deg: int              # absolute angle, whole degrees
    tgt_deg: int
    err_deg: int
    isense: int               # raw ISENSE ADC count
    moving: bool              # True=MOV, False=HLD
    fault: bool

    @property
    def logical_gear(self) -> int:
        """1-based gear the rider would name (internal index + 1)."""
        return self.gear + 1


@dataclass(frozen=True)
class GearTable:
    """The 12 positions the firmware prints (p0..p11); logical gears 1..11 use p0..p10.

    Values are whole degrees (the firmware's stored ``int32`` gear_pos).
    """

    positions_deg: tuple[int, ...]

    def position_for(self, logical_gear: int) -> int:
        """Absolute hold angle (deg) for a 1-based logical gear (1..11)."""
        if not 1 <= logical_gear <= NUM_REAR_GEARS:
            raise ValueError(f"logical gear {logical_gear} out of range 1..{NUM_REAR_GEARS}")
        return self.positions_deg[logical_gear - 1]

    def position_for_index(self, gear_index: int) -> int:
        """Absolute hold angle (deg) for a 0-based internal gear index (0..10)."""
        return self.position_for(gear_index + 1)

    def is_calibrated(self) -> bool:
        """True if the used entries look like a real fit (not the all-zero sentinel)."""
        used = self.positions_deg[:NUM_REAR_GEARS]
        if len(used) < NUM_REAR_GEARS:
            return False
        if all(p == 0 for p in used):
            return False
        # A valid affine fit through two refs is strictly monotonic across gears.
        diffs = [b - a for a, b in zip(used, used[1:])]
        return all(d > 0 for d in diffs) or all(d < 0 for d in diffs)


@dataclass(frozen=True)
class TelemetryRow:
    """One CSV telemetry row. Angles are centi-degrees; ``current_cd`` is absolute."""

    t_ms: int
    target_cd: int
    current_cd: int
    error_cd: int
    drive: int
    integral_cd: int
    state: int                # 0=HLD, 1=MOV
    isense: int
    fault: int                # 0/1

    @property
    def moving(self) -> bool:
        return self.state == 1


@dataclass(frozen=True)
class Event:
    """A parsed ``#tag,key=value,...`` HIL event, host-timestamped on arrival."""

    kind: str                 # 'boot', 'shift', 'turn', 'save', 'cal', 'fault', 'reboot'
    fields: dict[str, Any]
    host_ts: float            # perf_counter() at read time
    raw: str


@dataclass
class MoveMetrics:
    """Per-move metrics. Times are milliseconds (device clock); angles degrees."""

    label: str
    final_target_deg: float
    start_deg: float
    distance_deg: float                 # signed final - start
    time_to_move_ms: float | None       # first-motion latency
    time_to_target_ms: float | None     # first entry into settle band
    settle_time_fw_ms: float | None     # MOV->HLD state edge
    settle_time_band_ms: float | None   # last sustained band entry
    overshoot_deg: float | None
    overshoot_pct: float | None
    steady_state_err_deg: float | None
    peak_isense: int | None
    hold_isense: float | None
    turns_traversed: int
    turn_events: int                    # counted '#turn' events in the window
    n_rows: int
    flags: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        # NaN is not valid JSON; coerce to None for portable logs.
        for k, v in d.items():
            if isinstance(v, float) and math.isnan(v):
                d[k] = None
        return d


@dataclass
class CalibrationResult:
    low_ref_deg: int | None
    high_ref_deg: int | None
    span_deg: float | None
    gear_table: GearTable | None
    ok: bool
    flags: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "low_ref_deg": self.low_ref_deg,
            "high_ref_deg": self.high_ref_deg,
            "span_deg": self.span_deg,
            "gear_table_deg": list(self.gear_table.positions_deg) if self.gear_table else None,
            "ok": self.ok,
            "flags": self.flags,
        }
