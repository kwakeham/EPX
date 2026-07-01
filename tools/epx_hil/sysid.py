"""Open-loop plant system-ID primitives (reused by ``characterize`` and autotune).

Drives the motor open-loop via the firmware ``u`` command (PID bypassed, ~250 ms
watchdog) and derives a plant model from the telemetry response:

* **breakaway drive** ``D_c`` — the duty at which motion starts (Coulomb/stiction).
* **viscous friction** ``b`` — slope of drive vs steady velocity [duty per deg/s].
* **time constant** ``tau`` and **effective inertia** ``J = b·tau`` — from a step.

From that, suggested seed gains for the firmware PID (``Kp`` duty/deg,
``Kd`` duty/(deg/s), ``Ki`` duty/(deg·s)), to be refined closed-loop by autotune.

Everything the loop applies is echoed back on the telemetry ``drive`` column, so
(drive, velocity) pairs come straight from the stream — no need to trust the
commanded value. Velocity is differentiated from the *absolute* ``current`` angle,
so it is clean across turn boundaries.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass

from . import protocol
from .link import SerialLink
from .models import TelemetryRow
from .units import cd_to_deg

WATCHDOG_S = 0.25            # firmware open-loop watchdog window


@dataclass
class PlantModel:
    breakaway_pos: float | None      # duty to start moving, + direction
    breakaway_neg: float | None
    viscous_b: float | None          # duty per (deg/s)
    coulomb_dc: float | None         # duty intercept (drive at v->0)
    tau_s: float | None              # velocity time constant
    inertia_j: float | None          # b * tau  [duty·s^2/deg]
    friction_r2: float | None
    n_points: int

    def to_dict(self) -> dict:
        return asdict(self)


# --------------------------------------------------------------------------- #
# open-loop capture
# --------------------------------------------------------------------------- #
def run_open_loop(
    link: SerialLink,
    drive_of_t,
    duration_s: float,
    *,
    divider: int = 2,
    refresh_hz: float = 15.0,
    isense_abort: int | None = None,
) -> list[TelemetryRow]:
    """Apply ``drive_of_t(elapsed_s)`` open-loop for ``duration_s``; capture telemetry.

    Refreshes ``u`` faster than the watchdog so the motor keeps driving, aborts on
    a fault (or ISENSE over ``isense_abort``), and always releases to a safe hold
    (stop refreshing -> firmware watchdog holds position).
    """
    state = {"fault": False, "over": False, "last": None}

    def on_row(r: TelemetryRow):
        state["last"] = r
        if r.fault:
            state["fault"] = True
        if isense_abort is not None and r.isense >= isense_abort:
            state["over"] = True

    link.set_row_callback(on_row)
    link.start_tele_capture()
    link.send_nowait(protocol.cmd_telemetry(divider))
    try:
        t0 = time.perf_counter()
        period = 1.0 / refresh_hz
        while True:
            elapsed = time.perf_counter() - t0
            if elapsed >= duration_s or state["fault"] or state["over"]:
                break
            link.send_nowait(protocol.cmd_open_loop(int(drive_of_t(elapsed))))
            time.sleep(period)
    finally:
        link.send_nowait(protocol.cmd_open_loop(0))     # stop driving
        link.send_nowait(protocol.cmd_telemetry(0))
        time.sleep(WATCHDOG_S + 0.1)                     # let the watchdog hold
        rows = link.stop_tele_capture()
        link.set_row_callback(None)
    if state["fault"]:
        raise SysIdFault("motor fault during open-loop run")
    if state["over"]:
        raise SysIdFault(f"ISENSE exceeded abort threshold during open-loop run")
    return rows


class SysIdFault(RuntimeError):
    pass


# --------------------------------------------------------------------------- #
# analysis
# --------------------------------------------------------------------------- #
def velocity_series(rows: list[TelemetryRow], window: int = 4):
    """Return ``[(t_s, drive, vel_deg_s), ...]`` differentiating the absolute angle."""
    out = []
    for i in range(len(rows) - window):
        a, b = rows[i], rows[i + window]
        dt = (b.t_ms - a.t_ms) / 1000.0
        if dt <= 0:
            continue
        vel = (cd_to_deg(b.current_cd) - cd_to_deg(a.current_cd)) / dt
        # drive at the midpoint of the window (applied open-loop duty)
        drive = rows[i + window // 2].drive
        out.append(((a.t_ms + b.t_ms) / 2000.0, drive, vel))
    return out


def _linfit(xs, ys):
    """Least-squares y = a + b*x -> (a, b, r2). Returns (None, None, None) if degenerate."""
    n = len(xs)
    if n < 2:
        return None, None, None
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-9:
        return None, None, None
    b = (n * sxy - sx * sy) / denom
    a = (sy - b * sx) / n
    mean_y = sy / n
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else None
    return a, b, r2


def friction_fit(rows: list[TelemetryRow], vel_min_deg_s: float = 15.0):
    """Fit drive = D_c + b*velocity over the moving portion (one direction).

    Returns ``(coulomb_dc, viscous_b, r2, n)``. Uses only samples where the motor
    is actually turning (|vel| >= vel_min) so stiction/standstill points don't bias
    the line; signs are folded so it works for either drive direction.
    """
    pts = [(abs(v), abs(d)) for (_, d, v) in velocity_series(rows) if abs(v) >= vel_min_deg_s]
    if len(pts) < 4:
        return None, None, None, len(pts)
    vs = [p[0] for p in pts]
    ds = [p[1] for p in pts]
    dc, b, r2 = _linfit(vs, ds)
    return dc, b, r2, len(pts)


def breakaway_from_ramp(rows: list[TelemetryRow], vel_thresh_deg_s: float = 20.0):
    """First applied |drive| at which |velocity| exceeds a threshold (a ramp run)."""
    for (_, d, v) in velocity_series(rows):
        if abs(v) >= vel_thresh_deg_s:
            return abs(d)
    return None


def tau_from_step(rows: list[TelemetryRow], settle_frac: float = 0.4):
    """Estimate the velocity time constant from a step: time to 63.2% of steady vel.

    Returns ``(tau_s, v_ss_deg_s)`` or ``(None, None)``.
    """
    vs = velocity_series(rows)
    if len(vs) < 8:
        return None, None
    t0 = vs[0][0]
    tail = vs[int(len(vs) * (1 - settle_frac)):]
    v_ss = sum(v for (_, _, v) in tail) / len(tail)
    if abs(v_ss) < 1e-3:
        return None, None
    target = 0.632 * v_ss
    for (t, _, v) in vs:
        if (v_ss > 0 and v >= target) or (v_ss < 0 and v <= target):
            return max(t - t0, 1e-3), v_ss
    return None, v_ss


@dataclass
class CharacterizeConfig:
    ramp_max_drive: int = 150
    ramp_duration_s: float = 4.0
    step_drive: int = 120
    step_duration_s: float = 1.5
    divider: int = 2                 # 128 Hz: safe under 115200 baud
    refresh_hz: float = 15.0
    isense_abort: int | None = None
    vel_thresh_deg_s: float = 20.0
    vel_min_fit_deg_s: float = 15.0
    settle_between_s: float = 0.4


def _avg(*vals):
    xs = [v for v in vals if v is not None]
    return sum(xs) / len(xs) if xs else None


def characterize(link: SerialLink, cfg: CharacterizeConfig | None = None, log=print):
    """Run the full open-loop characterization. Returns ``(PlantModel, runs)``.

    ``runs`` holds the raw row lists (``up``/``dn``/``step``) for logging/plots.
    Assumes the caller has cleared any latched fault.
    """
    cfg = cfg or CharacterizeConfig()

    def ramp(sign):
        return run_open_loop(
            link,
            lambda t: sign * cfg.ramp_max_drive * min(1.0, t / cfg.ramp_duration_s),
            cfg.ramp_duration_s, divider=cfg.divider, refresh_hz=cfg.refresh_hz,
            isense_abort=cfg.isense_abort)

    log("  ramp + (breakaway / friction)")
    up = ramp(+1)
    time.sleep(cfg.settle_between_s)
    log("  ramp - (breakaway / friction)")
    dn = ramp(-1)
    time.sleep(cfg.settle_between_s)

    ba_pos = breakaway_from_ramp(up, cfg.vel_thresh_deg_s)
    ba_neg = breakaway_from_ramp(dn, cfg.vel_thresh_deg_s)
    dc_p, b_p, r2_p, n_p = friction_fit(up, cfg.vel_min_fit_deg_s)
    dc_n, b_n, r2_n, n_n = friction_fit(dn, cfg.vel_min_fit_deg_s)
    b = _avg(b_p, b_n)
    dc = _avg(dc_p, dc_n)
    r2 = _avg(r2_p, r2_n)

    log("  step (inertia / time constant)")
    step = run_open_loop(link, lambda t: cfg.step_drive, cfg.step_duration_s,
                         divider=cfg.divider, refresh_hz=cfg.refresh_hz,
                         isense_abort=cfg.isense_abort)
    tau, _v_ss = tau_from_step(step)
    j = (b * tau) if (b is not None and tau is not None) else None

    model = PlantModel(breakaway_pos=ba_pos, breakaway_neg=ba_neg, viscous_b=b,
                       coulomb_dc=dc, tau_s=tau, inertia_j=j, friction_r2=r2,
                       n_points=(n_p + n_n))
    return model, {"up": up, "dn": dn, "step": step}


def suggest_gains(model: PlantModel, wn_rad_s: float, zeta: float = 1.0,
                  ki_ratio: float = 8.0) -> dict | None:
    """Seed PID gains from the plant for a target bandwidth ``wn`` and damping ``zeta``.

    Second-order motion model ``J x'' + b x' = u`` with ``u = Kp e - Kd x'`` gives
    ``Kp = J*wn^2``, ``Kd = 2*zeta*wn*J - b``; ``Ki`` is seeded slow (``Kp*wn/ratio``)
    and MUST be refined closed-loop (limit-cycle risk against stiction).
    """
    if model.inertia_j is None or model.viscous_b is None:
        return None
    j, b = model.inertia_j, model.viscous_b
    kp = j * wn_rad_s * wn_rad_s
    kd = max(0.0, 2.0 * zeta * wn_rad_s * j - b)
    ki = kp * wn_rad_s / ki_ratio
    return {"wn_rad_s": wn_rad_s, "zeta": zeta,
            "Kp": round(kp, 4), "Ki": round(ki, 4), "Kd": round(kd, 4)}
