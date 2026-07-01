"""EPX hardware-in-the-loop (HIL) validation harness.

Drives the EPX motor-controller console (UART 115200 8N1 over the J-Link CDC
port), captures the firmware's CSV telemetry and ``#event`` HIL log, runs a
repeatable test battery, and writes timestamped, never-overwritten logs with
per-move metrics.

Layout (leaf -> composite):
    units, models, protocol   pure: conversions, dataclasses, command/parse grammar
    link, probe               serial reader-thread + line demux; COM autodetect
    telemetry, moves          per-move capture context + logged-move orchestration
    metrics                   pure metric math over a captured window
    calibrate, reset          two-point calibration FSM; reboot + recovery check
    logging_, report          log-folder layout + summary tables
    cli, __main__, offline    argparse CLI; transcript replay (no hardware)
    tests/                    the HIL test battery (registry + suites)

The pure modules (units/models/protocol/metrics/report/offline) import and run
with no hardware, so parsing and metrics are unit-testable and old logs can be
re-scored.
"""

__version__ = "0.1.0"
