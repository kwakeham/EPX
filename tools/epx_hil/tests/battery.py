"""The HIL test battery: four suites registered for ``test <name>`` / ``run-all``.

Each suite logs its moves (via :mod:`epx_hil.moves`) and records non-move
verdicts via ``ctx.session.add_check`` so everything lands in ``summary.md`` and
the exit code. Suites read tunable knobs off ``ctx.args`` with conservative
defaults so a bare ``run-all`` stays reasonably quick and safe.
"""

from __future__ import annotations

import statistics
import time

from .. import commands, moves, protocol, reset
from ..models import NUM_REAR_GEARS
from ..telemetry import capture_move, wait_settle
from ..units import cd_to_deg
from .registry import hil_test

TICK_MS = 1000.0 / 256.0  # control-loop period (256 Hz)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _have_table(ctx) -> bool:
    if ctx.gear_table and ctx.gear_table.is_calibrated():
        return True
    ctx.log("  gear table not calibrated; skipping suite.")
    ctx.session.add_check("gear table calibrated", False, "run 'calibrate' first")
    return False


def _gear_move(ctx, logical: int, label: str | None = None):
    return moves.run_gear_move(ctx.link, ctx.session, ctx.gear_table, logical - 1,
                               label=label, divider=ctx.tele_div, settle_timeout=ctx.settle_timeout)


def _settled_pos(ctx):
    st = commands.read_status(ctx.link)
    return st.pos_deg if st else None


def _reboot(ctx):
    method = getattr(ctx.args, "method", "r") or "r"
    link, rb = reset.reboot(ctx.link, method, baud=ctx.args.baud,
                            transcript_cb=ctx.session.transcript_cb, timeout=15.0, log=ctx.log)
    ctx.link = link  # method 'nrfjprog' returns a fresh link on a re-enumerated port
    return rb


# --------------------------------------------------------------------------- #
# core motion
# --------------------------------------------------------------------------- #
@hil_test("core-motion", "full-cassette sweep + repeatability/hysteresis")
def core_motion(ctx):
    if not _have_table(ctx):
        return
    log = ctx.log

    log("  stepwise sweep up then down")
    order = list(range(1, NUM_REAR_GEARS + 1)) + list(range(NUM_REAR_GEARS - 1, 0, -1))
    for logical in order:
        _gear_move(ctx, logical, label=f"sweep g{logical}")

    target = min(6, NUM_REAR_GEARS - 1)
    reps = getattr(ctx.args, "repeat", None) or 3
    log(f"  repeatability/hysteresis at gear {target} x{reps}")
    below, above = [], []
    for i in range(reps):
        _gear_move(ctx, target - 1)                       # sit below
        _gear_move(ctx, target, label=f"rep{i} below->g{target}")
        p = _settled_pos(ctx)
        if p is not None:
            below.append(p)
        _gear_move(ctx, target + 1)                       # sit above
        _gear_move(ctx, target, label=f"rep{i} above->g{target}")
        p = _settled_pos(ctx)
        if p is not None:
            above.append(p)

    samples = below + above
    if len(samples) >= 2:
        sigma = statistics.pstdev(samples)
        hyst = (statistics.mean(above) - statistics.mean(below)) if below and above else float("nan")
        rep_ok = sigma <= (getattr(ctx.args, "rep_sigma_max", None) or 5.0)
        ctx.session.add_check("repeatability", rep_ok,
                              f"sigma={sigma:.2f}deg hysteresis={hyst:.2f}deg n={len(samples)}")
        log(f"  repeatability sigma={sigma:.2f}deg hysteresis={hyst:.2f}deg")
    else:
        ctx.session.add_check("repeatability", False, "insufficient settled samples")


# --------------------------------------------------------------------------- #
# safety
# --------------------------------------------------------------------------- #
@hil_test("safety", "overcurrent latch+recover, power-loss recovery, boot-slam")
def safety(ctx):
    if not _have_table(ctx):
        return
    _overcurrent(ctx)
    _power_loss_and_boot_slam(ctx)


def _overcurrent(ctx):
    log = ctx.log
    # Reference move to learn the normal drive-current peak, then set the ISENSE
    # limit just below it so the next (normal) move trips the latch -- proving the
    # protection without needing a physical stall.
    ref = _gear_move(ctx, 1, label="oc ref g->g1")
    peak = ref.peak_isense
    if not peak or peak <= 0:
        ctx.session.add_check("overcurrent latch", False, "no ISENSE reading on reference move")
        return
    trip = max(1, int(peak * 0.7))
    log(f"  overcurrent: ref peak={peak}, setting limit={trip} count=3")
    commands.set_isense_limit(ctx.link, trip)
    commands.set_isense_count(ctx.link, 3)

    with capture_move(ctx.link, divider=ctx.tele_div, tag="overcurrent") as win:
        time.sleep(0.1)
        win.t_cmd = ctx.link.send_nowait(protocol.cmd_shift_to_index(NUM_REAR_GEARS - 1))
        res = wait_settle(ctx.link, timeout=ctx.settle_timeout)
        win.fault = res.fault
    faulted = (res.fault or any(r.fault for r in win.rows)
               or any(e.kind == "fault" for e in win.events))
    ctx.session.add_check("overcurrent latch", faulted, f"limit={trip} peak_ref={peak}")

    cleared = commands.clear_fault(ctx.link)
    st = commands.read_status(ctx.link)
    ctx.session.add_check("overcurrent clear", cleared and st is not None and not st.fault, "")

    # Restore a protective-but-non-nuisance limit (~3x normal peak). The harness
    # cannot read the original stored value, so this is logged loudly.
    restore = int(peak * 3) + 100
    commands.set_isense_limit(ctx.link, restore)
    commands.set_isense_count(ctx.link, 5)
    log(f"  overcurrent: restored ISENSE limit={restore} count=5 "
        f"(harness changed the stored limit; re-tune 'xl'/'xc' if a specific value is required)")


def _power_loss_and_boot_slam(ctx):
    log = ctx.log
    _gear_move(ctx, NUM_REAR_GEARS, label="pre-reboot ->g11")  # a multi-turn position
    log("  power-loss recovery: reboot mid-hold")
    rb = _reboot(ctx)
    ctx.session.add_check("power-loss recovery", rb.recovered, rb.message)

    # Boot-slam: post-boot the controller must NOT fling toward a stale target.
    with capture_move(ctx.link, divider=ctx.tele_div, tag="boot-slam") as win:
        time.sleep(1.2)
    if win.rows:
        base = win.rows[0].current_cd
        excursion = cd_to_deg(max(abs(r.current_cd - base) for r in win.rows))
        limit = getattr(ctx.args, "boot_slam_max", None) or 360.0
        ctx.session.add_check("boot-slam bounded", excursion <= limit,
                              f"excursion={excursion:.1f}deg (limit {limit:.0f})")
        log(f"  boot-slam excursion={excursion:.1f}deg")
    else:
        ctx.session.add_check("boot-slam bounded", False, "no telemetry after boot")


# --------------------------------------------------------------------------- #
# tuning
# --------------------------------------------------------------------------- #
@hil_test("tuning", "step-response/PID, holding current, timing jitter")
def tuning(ctx):
    _step_response(ctx)
    _hold_current(ctx)
    _timing_jitter(ctx)


def _step_response(ctx):
    st = commands.read_status(ctx.link)
    if st is None:
        ctx.session.add_check("step-response", False, "no status")
        return
    base = st.pos_deg
    ctx.log(f"  step-response from {base}deg (gains recorded in meta.json)")
    for delta in (90.0, 180.0):
        moves.run_angle_move(ctx.link, ctx.session, base + delta, label=f"step +{int(delta)}",
                             divider=ctx.tele_div, settle_timeout=ctx.settle_timeout)
        moves.run_angle_move(ctx.link, ctx.session, base, label=f"step -{int(delta)}",
                             divider=ctx.tele_div, settle_timeout=ctx.settle_timeout)


def _hold_current(ctx):
    with capture_move(ctx.link, divider=ctx.tele_div, tag="hold-current") as win:
        time.sleep(2.0)
    hold = [r.isense for r in win.rows if not r.moving]
    if hold:
        mean_i = statistics.mean(hold)
        ctx.session.add_check("holding current", True, f"mean_isense={mean_i:.0f} n={len(hold)}")
        ctx.log(f"  holding current mean_isense={mean_i:.0f} ({len(hold)} HLD rows)")
    else:
        ctx.session.add_check("holding current", False, "no HLD telemetry rows")


def _timing_jitter(ctx):
    with capture_move(ctx.link, divider=ctx.tele_div, tag="jitter") as win:
        time.sleep(1.5)
    ts = [r.t_ms for r in win.rows]
    deltas = [b - a for a, b in zip(ts, ts[1:])]
    if not deltas:
        ctx.session.add_check("timing jitter", False, "no telemetry rows")
        return
    expected = ctx.tele_div * TICK_MS
    within = sum(1 for d in deltas if abs(d - expected) <= TICK_MS)
    frac = within / len(deltas)
    ctx.session.add_check("timing jitter", frac >= 0.99,
                          f"{within}/{len(deltas)} within ±1 tick, exp={expected:.1f}ms")
    ctx.log(f"  timing jitter {within}/{len(deltas)} within ±1 tick (exp {expected:.1f}ms)")


# --------------------------------------------------------------------------- #
# endurance + flash
# --------------------------------------------------------------------------- #
@hil_test("endurance", "g1<->g11 cycling + flash-persistence stress")
def endurance(ctx):
    if not _have_table(ctx):
        return
    cycles = getattr(ctx.args, "cycles", None) or 5
    ctx.log(f"  endurance: {cycles} g1<->g11 cycles")
    for i in range(cycles):
        _gear_move(ctx, 1, label=f"cyc{i} ->g1")
        _gear_move(ctx, NUM_REAR_GEARS, label=f"cyc{i} ->g11")
    _flash_persistence(ctx)


def _flash_persistence(ctx):
    log = ctx.log
    before_tbl = commands.read_gear_table(ctx.link)
    before_gains = commands.read_gains(ctx.link)
    log("  flash persistence: force save x3 then reboot")
    for _ in range(3):
        ctx.link.send_nowait(protocol.cmd_force_save())
        time.sleep(0.25)
    _reboot(ctx)
    after_tbl = commands.read_gear_table(ctx.link)
    after_gains = commands.read_gains(ctx.link)
    same = bool(before_tbl and after_tbl
                and before_tbl.positions_deg == after_tbl.positions_deg
                and before_gains == after_gains)
    ctx.session.add_check("flash persistence", same,
                          "gear table + gains identical across reboot" if same
                          else "config changed across reboot")
