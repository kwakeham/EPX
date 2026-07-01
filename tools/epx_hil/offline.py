"""Offline (no-hardware) helpers: replay a transcript, re-score a move CSV.

Feeds recorded lines through the exact same :func:`protocol.classify_line` /
parsers and :func:`metrics.compute` used live, so parsing + metrics are testable
against checked-in samples and old logs can be re-scored after a formula tweak.
"""

from __future__ import annotations

import csv
from pathlib import Path

from . import metrics, protocol
from .models import Event, LineTag, MoveMetrics, TelemetryRow


def read_move_csv(path: str | Path) -> list[TelemetryRow]:
    """Load a ``moves/NNNN_*.csv`` file into telemetry rows."""
    rows: list[TelemetryRow] = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header != protocol.CSV_HEADER.split(","):
            # Tolerate: only proceed if it still looks like the 9-col telemetry.
            if header is None or len(header) != 9:
                raise ValueError(f"{path}: not an EPX telemetry CSV")
        for r in reader:
            if len(r) != 9:
                continue
            try:
                rows.append(TelemetryRow(*(int(x) for x in r)))
            except ValueError:
                continue
    return rows


def rescore(csv_path: str | Path, final_target_deg: float, **kw) -> MoveMetrics:
    """Recompute :class:`MoveMetrics` for a saved move CSV."""
    from .units import deg_to_cd
    rows = read_move_csv(csv_path)
    label = Path(csv_path).stem
    return metrics.compute(label, rows, deg_to_cd(final_target_deg), **kw)


def replay_transcript(path: str | Path) -> dict:
    """Re-route a transcript.log's RX lines through the demux; return a summary."""
    events: list[Event] = []
    n_rows = 0
    replies: list[str] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            # transcript format: "<ts> RX <tag> <text...>" (text may contain spaces)
            parts = line.rstrip("\n").split(None, 3)
            if len(parts) < 4 or parts[1] != "RX":
                continue
            text = parts[3]
            tag = protocol.classify_line(text)
            if tag is LineTag.EVENT:
                ev = protocol.parse_event(text)
                if ev:
                    events.append(ev)
            elif tag is LineTag.CSV_ROW:
                n_rows += 1
            elif tag in (LineTag.STATUS, LineTag.REPLY, LineTag.BANNER):
                replies.append(text)
    return {"events": events, "row_count": n_rows, "replies": replies}
