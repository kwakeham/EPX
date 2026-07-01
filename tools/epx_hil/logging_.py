"""Log session: never-overwrite per-run folders, transcript, events, summaries.

Layout::

    logs/<YYYY-MM-DD>/<HHMMSS>_<label>/
        meta.json          git rev, port, banner, gains, gear table, cli args
        transcript.log     every TX/RX line, host-timestamped (all tags)
        events.jsonl       one JSON object per '#event' line
        moves/NNNN_<name>.csv        raw firmware CSV rows for the move
        moves/NNNN_<name>.meta.json  move params + MoveMetrics + flags
        calibration/cal.json
        summary.csv / summary.md
        plots/*.png        (optional; written by report/plot helpers)
"""

from __future__ import annotations

import csv
import json
import re
import threading
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

from . import report
from .models import Event, MoveMetrics, TelemetryRow
from .protocol import CSV_HEADER

_SAFE = re.compile(r"[^A-Za-z0-9_.-]+")


def _slug(s: str) -> str:
    return _SAFE.sub("_", s).strip("_") or "run"


class LogSession:
    """One timestamped run folder; thread-safe transcript/event writers."""

    def __init__(self, root: str | Path, label: str, now: datetime | None = None):
        now = now or datetime.now()
        day = now.strftime("%Y-%m-%d")
        stamp = now.strftime("%H%M%S")
        base = Path(root) / day
        base.mkdir(parents=True, exist_ok=True)
        name = f"{stamp}_{_slug(label)}"
        path = base / name
        n = 2
        while path.exists():  # never overwrite (two runs in the same second)
            path = base / f"{name}_{n}"
            n += 1
        path.mkdir(parents=True)

        self.dir = path
        self.moves_dir = path / "moves"
        self.moves_dir.mkdir()
        self._lock = threading.Lock()
        self._transcript = open(path / "transcript.log", "a", encoding="utf-8", buffering=1)
        self._events = open(path / "events.jsonl", "a", encoding="utf-8", buffering=1)
        self._records: list[dict] = []
        self._move_count = 0

    # ----- callbacks wired into the link ---------------------------------- #
    def transcript_cb(self, host_ts: float, direction: str, tag: str, text: str) -> None:
        arrow = "TX" if direction == "tx" else "RX"
        with self._lock:
            self._transcript.write(f"{host_ts:14.6f} {arrow} {tag:10} {text}\n")

    def on_event(self, ev: Event) -> None:
        with self._lock:
            self._events.write(json.dumps({
                "host_ts": ev.host_ts, "kind": ev.kind, "fields": ev.fields, "raw": ev.raw,
            }) + "\n")

    def attach(self, link) -> None:
        """Route the link's events into events.jsonl (transcript is wired at construction)."""
        link.set_event_callback(self.on_event)

    # ----- artifacts ------------------------------------------------------- #
    def write_meta(self, meta: dict) -> None:
        (self.dir / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")

    def add_move(self, name: str, rows: list[TelemetryRow], metrics: MoveMetrics,
                 params: dict | None = None) -> int:
        self._move_count += 1
        idx = self._move_count
        stem = f"{idx:04d}_{_slug(name)}"
        with open(self.moves_dir / f"{stem}.csv", "w", newline="", encoding="utf-8") as f:
            f.write(CSV_HEADER + "\n")
            w = csv.writer(f)
            for r in rows:
                w.writerow([r.t_ms, r.target_cd, r.current_cd, r.error_cd, r.drive,
                            r.integral_cd, r.state, r.isense, r.fault])
        (self.moves_dir / f"{stem}.meta.json").write_text(
            json.dumps({"index": idx, "name": name, "params": params or {},
                        "metrics": metrics.to_dict()}, indent=2), encoding="utf-8")
        self._records.append(report.record_for(idx, metrics))
        return idx

    def add_check(self, name: str, passed: bool, detail: str = "") -> int:
        """Record a non-move pass/fail check (fault latch, recovery, jitter, ...)."""
        self._move_count += 1
        idx = self._move_count
        rec = {c: "-" for c in report.SUMMARY_COLUMNS}
        rec["#"] = idx
        rec["move"] = name
        rec["flags"] = detail or "-"
        rec["result"] = "PASS" if passed else "FAIL"
        self._records.append(rec)
        return idx

    def write_calibration(self, result) -> None:
        d = self.dir / "calibration"
        d.mkdir(exist_ok=True)
        (d / "cal.json").write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")

    def write_summary(self) -> None:
        (self.dir / "summary.md").write_text(
            report.format_summary_md(self._records), encoding="utf-8")
        with open(self.dir / "summary.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=report.SUMMARY_COLUMNS)
            w.writeheader()
            for rec in self._records:
                w.writerow(rec)

    @property
    def records(self) -> list[dict]:
        return list(self._records)

    def any_failures(self) -> bool:
        return any(r["result"] == "FAIL" for r in self._records)

    def close(self) -> None:
        try:
            self.write_summary()
        finally:
            with self._lock:
                self._transcript.close()
                self._events.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
