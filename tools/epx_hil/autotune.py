"""Closed-loop PID autotune, seeded from the open-loop plant model (:mod:`sysid`).

This plant is stiction-dominated (breakaway drive >> what Kp can deliver at the
settle band), so the dominant tuning knob is **Ki** — it integrates until it
breaks stiction and kills the steady-state error. So autotune:

1. Sets a sensible **PD** from the plant (``Kp = J·wn²``, ``Kd = 2ζwnJ − b`` at
   ``wn = wn_factor/tau``) and leaves it fixed. With Ki=0 this always holds short
   of target (that's the stiction), which is expected.
2. Climbs a **Ki ladder**, measuring steady-state error and the **hold-hunt**
   amplitude (limit-cycle guard) over a dwell at hold. It stops when hunting
   appears, then picks the fastest-settling set that meets the ss target (or the
   lowest-ss set if none fully meets it).
3. Runs a **confirmation** move and leaves the winning gains applied (persisted).

Each trial applies gains over the console (``p``/``i``/``d``) and scores the move
with the same :mod:`metrics` as the rest of the harness.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from . import commands, metrics, protocol, sysid
from .link import SerialLink
from .telemetry import capture_move, wait_settle
from .units import cd_to_deg, deg_to_cd


@dataclass
class AutotuneConfig:
    step_deg: float = 90.0
    band_deg: float = 10.0
    ss_target_deg: float = 3.0
    hunt_limit_deg: float = 5.0
    settle_timeout: float = 6.0
    return_timeout: float = 3.0
    divider: int = 2
    zeta: float = 1.0
    wn_factor: float = 2.0                          # PD bandwidth = wn_factor / tau
    # x seed Ki. Kept modest: on an unloaded bench motor a too-high Ki winds up and
    # spins continuously (turn-crossing flash saves then stall the firmware), so we
    # ladder gently and stop on the first unstable/no-data trial.
    ki_ladder: tuple = (0.1, 0.2, 0.3, 0.4)
    hold_dwell_s: float = 0.7                        # dwell at hold before measuring
    hold_tail_s: float = 0.45                        # hunt window (inside the dwell)


@dataclass
class Trial:
    phase: str
    kp: float
    ki: float
    kd: float
    settle_ms: float | None
    overshoot_deg: float | None
    ss_err_deg: float | None
    hunt_deg: float
    settled: bool
    flags: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return self.__dict__


def _set_gains(link: SerialLink, kp: float, ki: float, kd: float) -> None:
    ok = lambda ln: protocol.parse_gains(ln) is not None
    link.send_and_await(protocol.cmd_set_kp(kp), expect=ok, timeout=1.0)
    link.send_and_await(protocol.cmd_set_ki(ki), expect=ok, timeout=1.0)
    link.send_and_await(protocol.cmd_set_kd(kd), expect=ok, timeout=1.0)


def _hold_hunt_deg(rows, tail_s: float) -> float:
    """Peak-to-peak position (deg) over the trailing hold window (limit-cycle size)."""
    if not rows:
        return 0.0
    t_end = rows[-1].t_ms
    tail = [r for r in rows if r.t_ms >= t_end - tail_s * 1000.0]
    if len(tail) < 2:
        tail = rows[-8:]
    cds = [r.current_cd for r in tail]
    return cd_to_deg(max(cds) - min(cds))


def _eval(link, ref_deg, kp, ki, kd, cfg, session, phase, label) -> Trial:
    _set_gains(link, kp, ki, kd)
    commands.ensure_angle_mode(link)
    link.send_nowait(protocol.cmd_set_angle(ref_deg))          # return to a common start
    commands.wait_settle_status(link, timeout=cfg.return_timeout)

    final_deg = ref_deg + cfg.step_deg
    with capture_move(link, divider=cfg.divider, tag=label) as win:
        time.sleep(0.1)
        win.t_cmd = link.send_nowait(protocol.cmd_set_angle(final_deg))
        res = wait_settle(link, timeout=cfg.settle_timeout)
        win.settled, win.fault = res.ok, res.fault
        time.sleep(cfg.hold_dwell_s)   # dwell so ss/hunt use steady hold data

    m = metrics.compute(label, win.rows, deg_to_cd(final_deg),
                        band_deg=cfg.band_deg, settled=win.settled, fault=win.fault)
    hunt = _hold_hunt_deg(win.rows, cfg.hold_tail_s)
    if session:
        session.add_move(label, win.rows, m, params={"kp": kp, "ki": ki, "kd": kd})
    return Trial(phase=phase, kp=kp, ki=ki, kd=kd,
                 settle_ms=m.settle_time_band_ms, overshoot_deg=m.overshoot_deg,
                 ss_err_deg=m.steady_state_err_deg, hunt_deg=hunt,
                 settled=win.settled, flags=list(m.flags))


def _as_best(t: Trial) -> dict:
    return {"kp": t.kp, "ki": t.ki, "kd": t.kd, "ss": t.ss_err_deg,
            "hunt": t.hunt_deg, "settle_ms": t.settle_ms, "settled": t.settled}


def autotune(link: SerialLink, plant: sysid.PlantModel,
             cfg: AutotuneConfig | None = None, session=None, log=print):
    """Return ``(best, trials)``; leaves the winning gains applied on the device."""
    cfg = cfg or AutotuneConfig()
    commands.clear_fault(link)
    commands.ensure_angle_mode(link)
    st = commands.read_status(link)
    if st is None:
        raise RuntimeError("no status; cannot autotune")
    ref = st.pos_deg
    tau = plant.tau_s or 0.05

    g = sysid.suggest_gains(plant, wn_rad_s=cfg.wn_factor / tau, zeta=cfg.zeta)
    if not g:
        raise RuntimeError("plant model lacks inertia/friction; run characterize first")
    kp, kd, ki_seed = g["Kp"], g["Kd"], max(g["Ki"], 0.5)
    log(f"  PD seed: Kp={kp} Kd={kd} (wn={cfg.wn_factor:g}/tau); Ki seed {ki_seed}")

    trials: list[Trial] = []
    cands: list[Trial] = []

    base = _eval(link, ref, kp, 0.0, kd, cfg, session, "base", "ki0")
    trials.append(base)
    log(f"    Ki=0 (PD only): ss={base.ss_err_deg} settle={base.settle_ms} hunt={base.hunt_deg:.1f}")

    for f in cfg.ki_ladder:
        ki = ki_seed * f
        t = _eval(link, ref, kp, ki, kd, cfg, session, "ki", f"ki{f:g}")
        trials.append(t)
        log(f"    Ki={ki:.2f}: ss={t.ss_err_deg} settle={t.settle_ms} "
            f"hunt={t.hunt_deg:.1f} settled={t.settled}")
        if t.ss_err_deg is None:
            # No telemetry captured -> the motor likely ran away (unloaded windup).
            # Stop before pushing Ki higher, back off to the last good gains.
            log("    -> no telemetry (possible runaway); stop laddering, backing off")
            commands.clear_fault(link)
            commands.ensure_angle_mode(link)
            link.send_nowait(protocol.cmd_set_angle(ref))
            break
        if t.hunt_deg > cfg.hunt_limit_deg:
            log("    -> hunting; stop laddering")
            break
        cands.append(t)

    # Prefer the fastest-settling set that meets the ss target; else lowest ss.
    meeters = [t for t in cands if t.settled and t.ss_err_deg is not None
               and t.ss_err_deg <= cfg.ss_target_deg]
    if meeters:
        pick = min(meeters, key=lambda t: t.settle_ms if t.settle_ms is not None else 9e9)
    elif cands:
        pick = min(cands, key=lambda t: t.ss_err_deg if t.ss_err_deg is not None else 999.0)
    else:
        pick = base  # nothing cleared the hunt gate; fall back to PD only

    best = _as_best(pick)
    _set_gains(link, best["kp"], best["ki"], best["kd"])
    conf = _eval(link, ref, best["kp"], best["ki"], best["kd"], cfg, session, "confirm", "confirm")
    trials.append(conf)
    best.update(confirm_ss=conf.ss_err_deg, confirm_settle_ms=conf.settle_ms,
                confirm_hunt=conf.hunt_deg, confirm_settled=conf.settled)
    log(f"  applied Kp={best['kp']} Ki={best['ki']} Kd={best['kd']}; "
        f"confirm ss={conf.ss_err_deg}deg settle={conf.settle_ms}ms settled={conf.settled}")
    return best, trials
