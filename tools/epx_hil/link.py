"""Serial link: one reader thread owns the port and demuxes the shared stream.

Command replies, ``#event`` lines and CSV telemetry rows are all interleaved on a
single UART, so a caller doing blocking ``readline()`` would race telemetry
against replies. Instead a daemon reader thread reads bytes, splits on newlines,
host-timestamps each line, writes it to the transcript, and routes it:

* ``#...``            -> the session event log (+ optional live callback)
* CSV header/row      -> the active telemetry capture buffer (+ optional callback)
* everything else     -> the reply queue that :meth:`SerialLink.send_and_await`
                         consumes.

Silent commands (``t``/``s``/``s+``/``s-``) send with :meth:`send_nowait` and are
verified by a follow-up ``?``.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import serial  # pyserial

from . import protocol
from .models import Event, LineTag, TelemetryRow

TranscriptCb = Callable[[float, str, str, str], None]  # (host_ts, direction, tag, text)


@dataclass
class Reply:
    """Result of :meth:`SerialLink.send_and_await`."""

    lines: list[str] = field(default_factory=list)
    matched: bool = False

    @property
    def text(self) -> str:
        return "\n".join(self.lines)

    def first(self) -> str | None:
        return self.lines[0] if self.lines else None


def _row_to_telemetry(t: tuple) -> TelemetryRow:
    return TelemetryRow(*t)


class SerialLink:
    """Owns a pyserial port + reader thread + demux."""

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        transcript: TranscriptCb | None = None,
        read_timeout: float = 0.05,
    ):
        self.port = port
        self.baud = baud
        self._transcript = transcript
        self._read_timeout = read_timeout

        self._ser: serial.Serial | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._reply_q: "queue.Queue[tuple[float, str]]" = queue.Queue()
        self._events: list[Event] = []
        self._tele_buf: list[TelemetryRow] | None = None
        self._parse_fails = 0

        self._row_cb: Callable[[TelemetryRow], None] | None = None
        self._event_cb: Callable[[Event], None] | None = None

        self.t0_perf = 0.0
        self.t0_wall = 0.0

    # ----- lifecycle ------------------------------------------------------- #
    def open(self) -> "SerialLink":
        self._ser = serial.Serial(
            self.port, self.baud, timeout=self._read_timeout,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        )
        self.t0_perf = time.perf_counter()
        self.t0_wall = time.time()
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, name="epx-reader", daemon=True)
        self._reader.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ----- reader thread --------------------------------------------------- #
    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError):
                break  # port yanked (COM re-enumeration); caller detects via is_alive()
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw = bytes(buf[:nl])
                del buf[: nl + 1]
                text = raw.decode("latin-1").rstrip("\r")
                if text == "":
                    continue
                self._route(time.perf_counter(), text)

    def _route(self, host_ts: float, text: str) -> None:
        tag = protocol.classify_line(text)
        if self._transcript:
            self._transcript(host_ts, "rx", tag.value, text)

        if tag is LineTag.EVENT:
            ev = protocol.parse_event(text, host_ts)
            if ev:
                with self._lock:
                    self._events.append(ev)
                if self._event_cb:
                    self._event_cb(ev)
            return

        if tag is LineTag.CSV_ROW:
            parsed = protocol.parse_csv_row(text)
            if parsed is None:
                with self._lock:
                    self._parse_fails += 1
                return
            row = _row_to_telemetry(parsed)
            with self._lock:
                if self._tele_buf is not None:
                    self._tele_buf.append(row)
            if self._row_cb:
                self._row_cb(row)
            return

        if tag is LineTag.CSV_HEADER:
            return  # header just confirms telemetry is live; nothing to store

        # STATUS / BANNER / REPLY all go to the reply queue for send_and_await.
        self._reply_q.put((host_ts, text))

    # ----- sending --------------------------------------------------------- #
    def _write_line(self, cmd: str) -> float:
        ts = time.perf_counter()
        if self._transcript:
            self._transcript(ts, "tx", "cmd", cmd)
        self._ser.write((cmd + "\r\n").encode("latin-1"))
        self._ser.flush()
        return ts

    def send_nowait(self, cmd: str) -> float:
        """Send a command that produces no reply; returns the host send timestamp."""
        return self._write_line(cmd)

    def _drain_replies(self) -> None:
        try:
            while True:
                self._reply_q.get_nowait()
        except queue.Empty:
            pass

    def send_and_await(
        self,
        cmd: str,
        expect: Callable[[str], bool] | None = None,
        until: Callable[[list[str]], bool] | None = None,
        timeout: float = 1.0,
        drain_first: bool = True,
    ) -> Reply:
        """Send ``cmd`` and collect reply lines until ``expect``/``until`` or timeout.

        * ``expect``: predicate on a single line; returns as soon as one matches.
        * ``until``: predicate on the accumulated line list (for multi-line replies
          like the two-line gear table); returns when it is satisfied.
        * neither: collects whatever arrives within ``timeout`` (matched=True if any).
        """
        if drain_first:
            self._drain_replies()
        self._write_line(cmd)
        deadline = time.perf_counter() + timeout
        reply = Reply()
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            try:
                _, line = self._reply_q.get(timeout=remaining)
            except queue.Empty:
                break
            reply.lines.append(line)
            if expect is not None and expect(line):
                reply.matched = True
                break
            if until is not None and until(reply.lines):
                reply.matched = True
                break
        if expect is None and until is None:
            reply.matched = bool(reply.lines)
        return reply

    # ----- telemetry capture + live callbacks ------------------------------ #
    def start_tele_capture(self) -> None:
        with self._lock:
            self._tele_buf = []

    def stop_tele_capture(self) -> list[TelemetryRow]:
        with self._lock:
            rows = self._tele_buf or []
            self._tele_buf = None
            return list(rows)

    def set_row_callback(self, cb: Callable[[TelemetryRow], None] | None) -> None:
        self._row_cb = cb

    def set_event_callback(self, cb: Callable[[Event], None] | None) -> None:
        self._event_cb = cb

    # ----- accessors ------------------------------------------------------- #
    def events_since(self, host_ts: float) -> list[Event]:
        with self._lock:
            return [e for e in self._events if e.host_ts >= host_ts]

    def all_events(self) -> list[Event]:
        with self._lock:
            return list(self._events)

    @property
    def parse_fails(self) -> int:
        return self._parse_fails

    def is_alive(self) -> bool:
        return bool(self._reader and self._reader.is_alive())
