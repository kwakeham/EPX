"""``python -m epx_hil <sub>`` — the HIL harness command line.

Subcommands run individually or compose into ``run-all``. Exit codes are
CI-friendly: 0 all-pass, 1 test failure(s), 2 device/comm error, 3 calibration
failure.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from . import calibrate, commands, moves, probe, protocol, reset
from .context import Context
from .link import SerialLink
from .logging_ import LogSession
from .models import NUM_REAR_GEARS

DEFAULT_LOGDIR = Path(__file__).resolve().parent / "logs"

EXIT_OK, EXIT_TESTFAIL, EXIT_COMM, EXIT_CALFAIL = 0, 1, 2, 3


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _log(verbose: bool):
    def log(msg: str) -> None:
        print(msg, flush=True)
    def vlog(msg: str) -> None:
        if verbose:
            print(msg, flush=True)
    log.v = vlog
    return log


def git_rev() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return "unknown"


def connect(args, session: LogSession | None = None) -> SerialLink:
    port = args.port or probe.autodetect(args.baud)
    transcript = session.transcript_cb if session else None
    link = SerialLink(port, args.baud, transcript=transcript).open()
    if session:
        session.attach(link)
    return link


def new_session(args, label: str) -> LogSession:
    return LogSession(args.logdir, label)


def collect_meta(link: SerialLink, args, extra: dict | None = None) -> dict:
    st = commands.read_status(link)
    gains = commands.read_gains(link)
    table = commands.read_gear_table(link)
    meta = {
        "git_rev": git_rev(),
        "port": link.port,
        "baud": args.baud,
        "status": st.__dict__ if st else None,
        "gains": {"Kp": gains[0], "Ki": gains[1], "Kd": gains[2]} if gains else None,
        "gear_table_deg": list(table.positions_deg) if table else None,
        "gear_table_calibrated": bool(table and table.is_calibrated()),
        "argv": sys.argv,
    }
    if extra:
        meta.update(extra)
    return meta


def require_gear_table(link: SerialLink, log):
    table = commands.read_gear_table(link)
    if table is None or not table.is_calibrated():
        log("ERROR: gear table is not calibrated (run 'calibrate' first).")
        return None
    return table


# --------------------------------------------------------------------------- #
# subcommands
# --------------------------------------------------------------------------- #
def cmd_ports(args) -> int:
    for p in probe.list_candidate_ports():
        vid = f"{p.vid:04X}" if p.vid else "----"
        answered = probe.probe_port(p.device, args.baud) if not args.no_probe else None
        tag = "" if answered is None else ("  <- EPX" if answered else "  (no reply)")
        print(f"{p.device:8} vid={vid} {p.description}{tag}")
    return EXIT_OK


def cmd_status(args) -> int:
    log = _log(args.verbose)
    try:
        link = connect(args)
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        return EXIT_COMM
    try:
        import time
        while True:
            st = commands.read_status(link)
            print(st if st else "(no status reply)", flush=True)
            if not args.watch:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    return EXIT_OK


def cmd_reset(args) -> int:
    log = _log(args.verbose)
    session = new_session(args, "reset")
    try:
        link = connect(args, session)
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        session.close()
        return EXIT_COMM
    try:
        link, result = reset.reboot(link, args.method, baud=args.baud,
                                    transcript_cb=session.transcript_cb, log=log)
        session.write_meta({"reboot": result.to_dict(), "git_rev": git_rev()})
        log(f"reboot: {result.message}")
        return EXIT_OK if result.recovered else EXIT_TESTFAIL
    finally:
        link.close()
        session.close()


def cmd_calibrate(args) -> int:
    log = _log(args.verbose)
    cfg = calibrate.CalibrationConfig(
        g2_delta_deg=args.g2_delta, g10_delta_deg=args.g10_delta,
        g2_angle_deg=args.g2_angle, g10_angle_deg=args.g10_angle,
        jog_step_deg=args.jog_step, settle_timeout=args.settle_timeout)
    try:
        link = connect(args, None if args.dry_jog else new_session(args, "calibrate"))
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        return EXIT_COMM
    cal = calibrate.Calibrator(link, cfg, log=log)
    try:
        if args.dry_jog:
            print(cal.plan())
            return EXIT_OK
        result = cal.run()
        log(f"calibration: {'OK' if result.ok else 'FAILED'} flags={result.flags}")
        return EXIT_OK if result.ok else EXIT_CALFAIL
    except calibrate.CalibrationError as e:
        log(f"CALIBRATION ERROR: {e}")
        return EXIT_CALFAIL
    finally:
        link.close()


def cmd_move(args) -> int:
    log = _log(args.verbose)
    session = new_session(args, args.label or "move")
    try:
        link = connect(args, session)
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        session.close()
        return EXIT_COMM
    try:
        session.write_meta(collect_meta(link, args))
        if args.to_gear is not None:
            table = require_gear_table(link, log)
            if table is None:
                return EXIT_CALFAIL
            m = moves.run_gear_move(link, session, table, args.to_gear - 1,
                                    divider=args.tele_div, settle_timeout=args.settle_timeout)
        else:
            m = moves.run_angle_move(link, session, args.to_angle,
                                     divider=args.tele_div, settle_timeout=args.settle_timeout)
        log(f"move '{m.label}': settle={m.settle_time_fw_ms}ms overshoot={m.overshoot_deg}deg "
            f"ss_err={m.steady_state_err_deg}deg flags={m.flags}")
        return EXIT_TESTFAIL if session.any_failures() else EXIT_OK
    finally:
        link.close()
        session.close()


def _sweep(link, session, table, lo, hi, step, reverse, repeat, args, log):
    seq = list(range(lo, hi + 1, step))
    order = (seq[::-1] if reverse else seq)
    for _ in range(repeat):
        prev = None
        for logical in order:
            m = moves.run_gear_move(link, session, table, logical - 1,
                                    divider=args.tele_div, settle_timeout=args.settle_timeout)
            log(f"  sweep g{logical}: settle={m.settle_time_fw_ms}ms "
                f"overshoot={m.overshoot_deg}deg flags={m.flags}")
            prev = logical


def cmd_sweep(args) -> int:
    log = _log(args.verbose)
    session = new_session(args, "sweep")
    try:
        link = connect(args, session)
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        session.close()
        return EXIT_COMM
    try:
        session.write_meta(collect_meta(link, args))
        table = require_gear_table(link, log)
        if table is None:
            return EXIT_CALFAIL
        _sweep(link, session, table, args.frm, args.to, args.step,
               args.reverse, args.repeat, args, log)
        # down as well unless a single direction requested
        if not args.reverse:
            _sweep(link, session, table, args.frm, args.to, args.step,
                   True, args.repeat, args, log)
        return EXIT_TESTFAIL if session.any_failures() else EXIT_OK
    finally:
        link.close()
        session.close()


def cmd_far(args) -> int:
    log = _log(args.verbose)
    session = new_session(args, "far")
    try:
        link = connect(args, session)
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        session.close()
        return EXIT_COMM
    try:
        session.write_meta(collect_meta(link, args))
        table = require_gear_table(link, log)
        if table is None:
            return EXIT_CALFAIL
        for _ in range(args.repeat):
            moves.run_gear_move(link, session, table, 0, label="g1->g11 (s0->s10)",
                                divider=args.tele_div, settle_timeout=args.settle_timeout)
            moves.run_gear_move(link, session, table, NUM_REAR_GEARS - 1, label="g11->g1 (s10->s0)",
                                divider=args.tele_div, settle_timeout=args.settle_timeout)
        return EXIT_TESTFAIL if session.any_failures() else EXIT_OK
    finally:
        link.close()
        session.close()


def cmd_test(args) -> int:
    from .tests import registry, battery  # noqa: F401  (battery registers suites on import)
    log = _log(args.verbose)
    if args.list or not args.name:
        for name, help_ in registry.list_tests():
            print(f"{name:16} {help_}")
        return EXIT_OK
    if args.name not in registry.names():
        log(f"unknown test '{args.name}'. Known: {', '.join(registry.names())}")
        return EXIT_TESTFAIL
    session = new_session(args, f"test_{args.name}")
    try:
        link = connect(args, session)
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        session.close()
        return EXIT_COMM
    try:
        table = commands.read_gear_table(link)
        session.write_meta(collect_meta(link, args))
        ctx = Context(link=link, session=session, gear_table=table, args=args,
                      log=log, settle_timeout=args.settle_timeout, tele_div=args.tele_div)
        registry.run(args.name, ctx)
        return EXIT_TESTFAIL if session.any_failures() else EXIT_OK
    finally:
        link.close()
        session.close()


def cmd_run_all(args) -> int:
    from .tests import registry, battery  # noqa: F401
    log = _log(args.verbose)
    session = new_session(args, args.label or "run-all")
    try:
        link = connect(args, session)
    except probe.NoDeviceError as e:
        log(f"ERROR: {e}")
        session.close()
        return EXIT_COMM
    try:
        session.write_meta(collect_meta(link, args))
        log("== status ==")
        log(str(commands.read_status(link)))

        if not args.skip_cal:
            log("== calibrate ==")
            cfg = calibrate.CalibrationConfig(settle_timeout=args.settle_timeout)
            res = calibrate.Calibrator(link, cfg, log=log).run()
            log(f"calibration: {'OK' if res.ok else 'FAILED'} {res.flags}")
            session.write_calibration(res)
            if not res.ok:
                log("calibration failed; aborting run-all.")
                return EXIT_CALFAIL

        table = require_gear_table(link, log)
        if table is None:
            return EXIT_CALFAIL

        log("== far moves (g1<->g11) ==")
        moves.run_gear_move(link, session, table, 0, label="g1->g11 (s0->s10)",
                            divider=args.tele_div, settle_timeout=args.settle_timeout)
        moves.run_gear_move(link, session, table, NUM_REAR_GEARS - 1, label="g11->g1 (s10->s0)",
                            divider=args.tele_div, settle_timeout=args.settle_timeout)

        ctx = Context(link=link, session=session, gear_table=table, args=args,
                      log=log, settle_timeout=args.settle_timeout, tele_div=args.tele_div)
        suites = args.tests.split(",") if args.tests else registry.names()
        for name in suites:
            if name not in registry.names():
                log(f"(skipping unknown test '{name}')")
                continue
            log(f"== test: {name} ==")
            try:
                registry.run(name, ctx)
            except Exception as e:  # a suite blowing up shouldn't sink the whole run
                log(f"test '{name}' raised: {e}")

        if not args.skip_reset:
            log("== reboot ==")
            link, rb = reset.reboot(link, args.method, baud=args.baud,
                                    transcript_cb=session.transcript_cb, log=log)
            log(f"reboot: {rb.message}")

        return EXIT_TESTFAIL if session.any_failures() else EXIT_OK
    finally:
        link.close()
        session.close()
        log(f"logs: {session.dir}")


def cmd_replay(args) -> int:
    from . import offline
    summary = offline.replay_transcript(args.path)
    print(f"replies={len(summary['replies'])} rows={summary['row_count']} "
          f"events={len(summary['events'])}")
    for ev in summary["events"]:
        print(f"  #{ev.kind} {ev.fields}")
    return EXIT_OK


# --------------------------------------------------------------------------- #
# argument parsing
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="epx_hil", description="EPX hardware-in-the-loop harness")
    p.add_argument("--port", help="serial port (default: autodetect J-Link CDC)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--logdir", default=str(DEFAULT_LOGDIR))
    p.add_argument("--label", default=None)
    p.add_argument("--tele-div", type=int, default=2, dest="tele_div",
                   help="telemetry divider (2=128Hz; 1=256Hz risks dropped frames)")
    p.add_argument("--event-mask", type=int, default=0x3F, dest="event_mask")
    p.add_argument("--settle-timeout", type=float, default=12.0, dest="settle_timeout")
    p.add_argument("--method", default="r", choices=["r", "nrfjprog"], help="reboot method")
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("ports", help="list + probe candidate COM ports")
    sp.add_argument("--no-probe", action="store_true", help="list only, do not open ports")
    sp.set_defaults(func=cmd_ports)

    sp = sub.add_parser("status", help="read one/continuous '?' status")
    sp.add_argument("--watch", action="store_true")
    sp.add_argument("--interval", type=float, default=0.5)
    sp.set_defaults(func=cmd_status)

    sp = sub.add_parser("reset", help="reboot + validate recovery")
    sp.set_defaults(func=cmd_reset)

    sp = sub.add_parser("calibrate", help="automated two-point calibration")
    sp.add_argument("--g2-delta", type=float, default=0.0, dest="g2_delta")
    sp.add_argument("--g10-delta", type=float, default=-5392.0, dest="g10_delta")
    sp.add_argument("--g2-angle", type=float, default=None, dest="g2_angle")
    sp.add_argument("--g10-angle", type=float, default=None, dest="g10_angle")
    sp.add_argument("--jog-step", type=float, default=90.0, dest="jog_step")
    sp.add_argument("--dry-jog", action="store_true", help="print the jog plan, do not move")
    sp.set_defaults(func=cmd_calibrate)

    sp = sub.add_parser("move", help="one logged move + metrics")
    g = sp.add_mutually_exclusive_group(required=True)
    g.add_argument("--to-gear", type=int, dest="to_gear", help="logical gear 1..11")
    g.add_argument("--to-angle", type=float, dest="to_angle", help="absolute angle (deg)")
    sp.set_defaults(func=cmd_move)

    sp = sub.add_parser("sweep", help="stepwise full-cassette sweep")
    sp.add_argument("--from", type=int, default=1, dest="frm")
    sp.add_argument("--to", type=int, default=NUM_REAR_GEARS, dest="to")
    sp.add_argument("--step", type=int, default=1)
    sp.add_argument("--reverse", action="store_true")
    sp.add_argument("--repeat", type=int, default=1)
    sp.set_defaults(func=cmd_sweep)

    sp = sub.add_parser("far", help="direct far moves g1<->g11")
    sp.add_argument("--repeat", type=int, default=1)
    sp.set_defaults(func=cmd_far)

    sp = sub.add_parser("test", help="run one named test suite")
    sp.add_argument("name", nargs="?", default=None)
    sp.add_argument("--list", action="store_true")
    sp.set_defaults(func=cmd_test)

    sp = sub.add_parser("run-all", help="full automatic battery")
    sp.add_argument("--skip-cal", action="store_true")
    sp.add_argument("--skip-reset", action="store_true")
    sp.add_argument("--tests", default=None, help="comma list (default: all registered)")
    sp.set_defaults(func=cmd_run_all)

    sp = sub.add_parser("replay", help="offline: summarise a transcript.log")
    sp.add_argument("path")
    sp.set_defaults(func=cmd_replay)

    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except probe.NoDeviceError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return EXIT_COMM
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return EXIT_COMM


if __name__ == "__main__":
    sys.exit(main())
