"""Single source of truth for angle-unit conversions.

The firmware speaks two angle scales on the same wire, so mixing them is the most
likely bug class in the harness:

* **CSV telemetry** ``target``/``current``/``error``/``integral`` are
  **centi-degrees** (firmware value * 100). ``current`` is the *absolute* angle,
  i.e. ``(current_rotations * 360 + within-turn) * 100``.
* **``?`` status** ``pos``/``tgt``/``err`` and the **gear table** are **whole
  degrees** (the firmware casts ``mpos_last_angle()`` / stores ``int32`` degrees).

Every conversion goes through here. Callers name variables ``*_cd`` (centi-deg)
or ``*_deg`` so the two never get silently added.
"""

from __future__ import annotations

import math

CENTI = 100.0
"""Centi-degree scale factor (telemetry fixed-point)."""

DEG_PER_TURN = 360.0


def cd_to_deg(cd: float) -> float:
    """Centi-degrees -> degrees."""
    return cd / CENTI


def deg_to_cd(deg: float) -> int:
    """Degrees -> centi-degrees (rounded to the firmware's integer fixed-point)."""
    return int(round(deg * CENTI))


def turns_at_deg(abs_deg: float) -> int:
    """Whole turns for an absolute angle in degrees (matches ``floor(abs/360)``).

    ``floor`` (not truncation) so the count is monotonic through negative angles,
    the same convention the firmware's ``current_rotations`` counter follows.
    """
    return math.floor(abs_deg / DEG_PER_TURN)


def turns_at_cd(abs_cd: float) -> int:
    """Whole turns for an absolute angle in centi-degrees."""
    return turns_at_deg(cd_to_deg(abs_cd))
