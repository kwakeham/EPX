"""Summary-table rendering (pure). Turns per-move metrics into summary.md/.csv rows.

A move's ``result`` is derived from its metric ``flags``: hard problems
(fault / settle timeout / no motion / never reached band) are FAIL, any other
flag is WARN, otherwise PASS.
"""

from __future__ import annotations

from .models import MoveMetrics

HARD_FLAGS = {"fault", "settle_timeout", "no_motion", "never_in_band", "no_rows"}

SUMMARY_COLUMNS = [
    "#", "move", "dist_deg", "t_move_ms", "t_settle_ms",
    "overshoot_deg", "overshoot_pct", "ss_err_deg", "peak_isense",
    "turns", "flags", "result",
]


def result_for(flags: list[str]) -> str:
    if any(f in HARD_FLAGS for f in flags):
        return "FAIL"
    return "WARN" if flags else "PASS"


def _fmt(v, nd: int = 1) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def record_for(index: int, m: MoveMetrics) -> dict:
    """Flatten one move's metrics into a summary record (order = SUMMARY_COLUMNS)."""
    settle = m.settle_time_fw_ms if m.settle_time_fw_ms is not None else m.settle_time_band_ms
    return {
        "#": index,
        "move": m.label,
        "dist_deg": _fmt(m.distance_deg),
        "t_move_ms": _fmt(m.time_to_move_ms),
        "t_settle_ms": _fmt(settle),
        "overshoot_deg": _fmt(m.overshoot_deg, 2),
        "overshoot_pct": _fmt(m.overshoot_pct, 2),
        "ss_err_deg": _fmt(m.steady_state_err_deg, 2),
        "peak_isense": _fmt(m.peak_isense),
        "turns": _fmt(m.turns_traversed),
        "flags": ",".join(m.flags) if m.flags else "-",
        "result": result_for(m.flags),
    }


def format_summary_md(records: list[dict]) -> str:
    """Render a GitHub-flavoured markdown table from summary records."""
    if not records:
        return "_(no moves logged)_\n"
    header = "| " + " | ".join(SUMMARY_COLUMNS) + " |"
    sep = "|" + "|".join("---" for _ in SUMMARY_COLUMNS) + "|"
    lines = [header, sep]
    for r in records:
        lines.append("| " + " | ".join(str(r.get(c, "")) for c in SUMMARY_COLUMNS) + " |")
    n_pass = sum(1 for r in records if r["result"] == "PASS")
    n_warn = sum(1 for r in records if r["result"] == "WARN")
    n_fail = sum(1 for r in records if r["result"] == "FAIL")
    lines.append("")
    lines.append(f"**{len(records)} moves — {n_pass} PASS, {n_warn} WARN, {n_fail} FAIL**")
    return "\n".join(lines) + "\n"
