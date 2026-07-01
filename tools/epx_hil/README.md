# EPX HIL harness

Hardware-in-the-loop validation for the EPX derailleur motor controller. Drives
the firmware console (UART 115200 8N1 over the J-Link CDC port), captures the
firmware's CSV telemetry + `#event` HIL log, runs a repeatable test battery, and
writes timestamped, never-overwritten logs with per-move metrics.

## Install

A venv keeps pyserial pinned to one interpreter (recommended on Windows, where the
`python` on PATH churns). From the `tools/` directory:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r epx_hil\requirements.txt   # + pytest to run offline tests
```

Run everything as a module from `tools/` (so `epx_hil` is importable). Windows
options:

```powershell
# direct-call (no activation, most reliable)
.\.venv\Scripts\python.exe -m epx_hil <subcommand> [options]

# or activate first, then use plain `python`
.\.venv\Scripts\Activate.ps1        # PowerShell (Set-ExecutionPolicy -Scope Process -Bypass if blocked)
.venv\Scripts\activate.bat          # cmd.exe
python -m epx_hil <subcommand>
```

On macOS/Linux: `source .venv/bin/activate` then `python -m epx_hil ...`.

## Quick start

```
python -m epx_hil ports                 # find/probe the J-Link CDC COM port
python -m epx_hil status                 # parsed '?' status
python -m epx_hil calibrate --dry-jog    # preview the calibration jog plan (no motion)
python -m epx_hil calibrate              # automated two-point calibration
python -m epx_hil move --to-gear 10      # one logged move + metrics (logical gear)
python -m epx_hil far                    # g1<->g11 far moves
python -m epx_hil reset                  # reboot (firmware 'r') + recovery check
python -m epx_hil run-all                # full battery: status -> cal -> far -> suites -> reboot
```

Global options: `--port COMx` (skip autodetect), `--baud`, `--logdir`, `--label`,
`--tele-div` (2 = 128 Hz; 1 = 256 Hz risks dropped frames at 115200),
`--settle-timeout`, `--method {r,nrfjprog}` (reboot), `-v`.

Exit codes: `0` pass, `1` test failure(s), `2` device/comm error, `3` calibration
failure — CI friendly.

## What it logs

Each run creates `logs/<YYYY-MM-DD>/<HHMMSS>_<label>/`:

- `meta.json` — git rev, port, `?`/`k`/`g` snapshots, cli args
- `transcript.log` — every TX/RX line, host-timestamped
- `events.jsonl` — parsed `#boot/#shift/#turn/#save/#cal/#fault` events
- `moves/NNNN_<name>.csv` + `.meta.json` — raw telemetry + computed `MoveMetrics`
- `calibration/cal.json`
- `summary.md` / `summary.csv`

Per-move metrics: time-to-move, time-to-target, settle time (firmware MOV→HLD edge
and computed band), overshoot vs the **final** gear position (deg + %), steady-state
error, peak/hold ISENSE, turns traversed (cross-checked against `#turn` events).

## Tests

Individual suites (`python -m epx_hil test <name>`, `test --list` to enumerate):

- **core-motion** — full-cassette stepwise sweep + repeatability/hysteresis
- **safety** — overcurrent latch+recover, power-loss recovery, boot-slam
- **tuning** — step-response/PID, holding current, timing jitter
- **endurance** — g1↔g11 cycling + flash-persistence stress

Offline (no hardware) unit tests for the parsers + metric math:

```
cd tools && python -m pytest epx_hil -q
```

## Design notes

- Angle scales differ on the wire: telemetry is centi-degrees (`current` is the
  *absolute* angle incl. rotations), `?` status and the gear table are whole
  degrees. All conversions go through `units.py`.
- Commands that read a fixed char index take no space (`ma`/`mg`/`gl`/`gh`/`gi`);
  `atoi`/`sscanf` ones tolerate spaces (`xl 5`, `o 5 0 ...`). See `protocol.py`.
- Direct shifts use the 0-based internal gear index: logical gear 1 = `s0`,
  gear 11 = `s10`. The CLI takes logical (1..11) and converts.
- Calibration captures gear 2 and gear 10 at two symmetric angles (each `--span`/2
  from the start). Jogs are bounded relative moves that wait for the position to
  *stop* (not for HLD, which never comes while a standing error exceeds the settle
  band), accept "stopped near target" as arrival, and abort to a safe hold only on
  a genuine stall far from target — so a bad target can't grind a real end stop.
- Reboot uses the firmware `r` command (the J-Link CDC survives an nRF reset, so
  the COM port persists); `--method nrfjprog` re-probes after re-enumeration.
