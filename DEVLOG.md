# EPX Engineering Log (DEVLOG)

A running engineering history so work is easy to follow across sessions. **Append an
entry per meaningful commit** (see "Commit history" at the bottom) and update the
relevant analysis/design sections when behavior changes. High-level project/build/flash
facts live in [CLAUDE.md](CLAUDE.md); this file is the narrative + decisions + open
threads.

---

## Session summary (through 2026-06-28)

EPX is a wireless motor controller for a Campagnolo EPS 11-speed derailleur (nRF52832 +
S132, nRF5 SDK 17). A DRV8874 H-bridge drives a DC motor; a sin/cos hall sensor gives
position. Recent work (newest first), all on `master`, pushed to `origin`:

- **Calibration fault-escape** (`f50ea34`): entering calibration (`c` / btn3-long) clears
  a latched overcurrent fault and holds the current position so you can jog off a hard
  stop. Verified on hardware.
- **Boot target from gear table + cal-jog direction** (`0d64e2d`): boot now seeks
  `gear_pos[current_gear]`; calibration Btn1/Btn2 direction corrected. **The boot-target
  half is the boot-slam regression — see below.**
- **Persist gear on shift** (`1a068c8`): a shift now requests a flash save immediately, so
  a reboot keeps the gear instead of reverting.
- **Multi-turn safety** (`3fb07ed`): save `current_rotations` whenever it changes; seed
  `angle_old` from the first real reading (fixes a first-sample miscount); documented the
  relative-actuator risk in CLAUDE.md.
- **Debug monitor + guided calibration** (`62d54f0`): console `?` one-shot status,
  `v<n>` low-rate human-readable monitor, `c` guided calibration with button jog/capture.
- **CLAUDE.md + recovered defaults** (`01fc7b6`): project doc + commit workflow; recovered
  historical tuning constants as comments.
- Earlier this effort (`8023035` "re-write based on new plan" and the prior session): PID
  rewritten to a reusable `pid_ctrl_t` (anti-windup, derivative-on-measurement, dt-scaled),
  motor sleep as an explicit `motor_sm` state machine, gear interpolation/profile fit,
  overshift/dwell sequencer, ISENSE/nFault overcurrent fault, UART console + CSV telemetry,
  config-version flash migration, the `flash_dfu` bootloader-settings fix, and the host SIL
  harness in `test/`.

**Active gains on the bench:** `Kp=1, Ki=0, Kd=0` (pure proportional → standing error;
needs tuning). Gear table is calibrated but the stored absolute position is currently
**lost/wrong**, which is what makes the boot-slam fire.

---

## Regression analysis — boot-slam into the hard stop (#1)

**Symptom:** on boot the derailleur drives hard into an end-of-travel stop and latches an
overcurrent fault (observed: `tgt -10708` while physically at `pos -9259`, err -1449).

**Root cause:** commit **`0d64e2d`** added `data_handler_set_boot_target()`
([libraries/data_handler.c](libraries/data_handler.c), called at [main.c:162](main.c#L162)),
which on boot sets the target to `gear_pos[current_gear]`. When the stored turn count has
drifted (relative actuator + abrupt power loss), that absolute target is far from the
physical position → a large blind move into the stop.

**Original behavior (`ca07aee`, worked):** boot ran `get_flash_values → mpos_init →
drv8874_init` with **no** boot-target step; the drive loop used `target =
link_epx_pos->target_angle`, i.e. the **last saved resting position**. On a clean reboot
that equals where the derailleur physically is → little/no motion. Safe.

**Suggested reversion:** remove the `data_handler_set_boot_target()` call (or change it to
hold the current *measured* position) so boot does **not** seek `gear_pos`. This restores
the safe original behavior and matches the agreed "boot = hold, not seek" safety principle
(below). This was a *requested* change (prior turn: "target angle can be ignored if
epx_configuration is filled in") that the later safety discussion showed to be unsafe — so
this is a deliberate direction change, not a careless bug.

**Keep (not regressions):**
- `3fb07ed` `angle_old` prime-from-first-reading: the original `angle_old = target_angle %
  360` had a latent bug — a *negative* saved target (e.g. -7359 % 360 = -159) vs a 0–360
  reading trips a false 180° wrap and decrements `current_rotations` on the first sample.
  The prime fix removes that. Keep it.
- `1a068c8` persist-gear-on-shift and `3fb07ed` save-on-rotation-change are additive
  (more-frequent saves). Keep them.

---

## Open design — safe position-loss recovery (UNRESOLVED — resume here)

Context: the sin/cos sensor is absolute only within one 360° turn; `current_rotations` is
the multi-turn count and is lost on abrupt power-off (see "Known risk" in CLAUDE.md). The
**rejected** idea was auto-home-to-hard-stop on boot — correctly vetoed because a reboot
mid-ride would fling the derailleur into the spokes/off the cassette (injury risk).

**Agreed safety principle:** the controller must never make a large or blind autonomous
move. A *position-uncertain* state should **freeze shifting and hold position** — worst
case "stuck in gear," never "throws the chain." All recovery is rider-initiated in a safe
context (on a stand), via calibration.

**Proposed mechanisms (no autonomous motion), pending decision:**
1. **Boot = hold, not seek** (= the regression reversion above).
2. **Soft limits:** clamp every commanded target to `gear_pos[0]…gear_pos[last]` ± margin
   so even a normal shift can't seek a hard stop.
3. **Plausibility / uncertainty flag:** if measured position disagrees with stored gear by
   more than ~1.5 cog spacings (or last shutdown was unsafe), enter "needs recalibration":
   shifting disabled, surfaced over console/BLE, cleared only by deliberate recalibration.
4. **Minimize loss:** frequent saves (done) + optionally a power-fail-comparator last-gasp
   save and/or hardware hold-up cap so the turn count is rarely lost.

**Unknowns needed from the user before building #3/#4:**
- **Mechanical ratio:** sensor turns across full gear-1→11 travel, and motor angle per cog.
  Live data suggested ~5 turns total and ~175°/cog ⇒ a lost turn ≈ 2 cogs ⇒ *detectable*
  by a plausibility check. Confirm the real numbers; if a turn < 1 cog, detection weakens.
- **Power/hold-up:** supply type and whether there's bulk/supercap capacitance on the rail
  (decides whether a POF last-gasp save is reliable).
- **"Is it safe to move?" gate:** preference is "never auto-move; only calibration does
  large moves" vs using the (unpopulated) LIS2DTW accelerometer to gate on stationary.
  Leaning "never."

---

## SIL test harness (`test/`) — status & what's needed (#4)

**What exists:** `test/sim_plant.c` (2nd-order motor model), `test/test_loop.c` (drives the
*real* `PID_controller`, `motor_sm`, `gears`, `shift_seq` against the plant; has unit
asserts + emits the same CSV columns as firmware telemetry), `test/Makefile` (host gcc),
`test/README.md`.

**Blocker to running it:** there is **no host C compiler** on the bench machine
(`gcc`/`cc`/`clang` all absent; only the cross `arm-none-eabi-gcc`). To use it, install one
of: MSYS2/MinGW-w64, or build under WSL, or run it in CI. Then `make -C test run`
(asserts → stderr, CSV → `run.csv`).

**To "complete" it for the test plan below, extend the model:**
- Add **hard end-stops** to `sim_plant` (clamp position; rising current/stall near a stop)
  so overcurrent and soft-limit behavior can be exercised.
- Model the **sin/cos within-turn angle + turn count** (wrap at 360°) so multi-turn
  counting and lost-turn scenarios are testable, not just a continuous position.
- Add a **simulated flash + reboot**: persist `{rotations, gear, target}` on the save
  triggers, then re-init the controller from that store to model power-loss/reboot.
- Optionally a tiny **scenario runner** so each test case in the plan is one function.

---

## Test plan (#5)

Each case is intended to run in the SIL first (pure logic), then be spot-checked on
hardware. "HW" notes the hardware procedure.

### Startup / boot
- **S1 Hold, no slam:** with a *wrong* stored turn count, boot must NOT drive into a stop
  (after the reversion: holds current position). HW: power-cycle, `?` shows small err, HLD.
- **S2 Resume valid gear:** with a *consistent* stored position, boot holds at the right
  gear (tgt ≈ pos). HW: settle in a gear, clean reboot, `?` matches.
- **S3 First-sample no miscount:** boot doesn't spuriously ±360 the turn count, incl. a
  negative saved target. HW: `?` pos stable across reset (verified for `3fb07ed`).

### Power loss / random reboot
- **P1 Reboot at rest:** position + gear exactly retained (saved on settle).
- **P2 Reboot mid-move (no turn crossing):** within-turn angle is read fresh; small error;
  no large move.
- **P3 Reboot mid-move across a turn boundary:** turn count saved on crossing keeps
  absolute position within one save-window; quantify worst-case error.
- **P4 Unsafe-shutdown flag (once built):** boot enters "needs recalibration", shifting
  disabled, no motion.

### Calibration
- **C1 Capture + interpolate:** capture gear 2 & 10, profile-fit fills 11 positions; persists.
- **C2 Jog direction:** Btn1=up / Btn2=down match physical derailleur motion (HW).
- **C3 Fault escape:** from a latched fault, `c` clears fault + holds; jogging works; a jog
  back into a stop re-faults but the next jog recovers (verified `f50ea34`).
- **C4 Re-cal restores validity:** after recalibration, S1/S2 pass again.

### Shifting
- **H1 Direction/magnitude:** up/down move the correct way by the gear spacing.
- **H2 Overshift+dwell:** target overshoots to the waypoint, dwells, settles; `overshift==0`
  ⇒ single move, no dwell.
- **H3 Multi-shift (button hold):** repeated LONG events step multiple gears, bounded to ends.
- **H4 Gear persisted on shift:** shift then immediate reboot keeps the new gear (`1a068c8`).

### Overcurrent / end-stop / limits
- **O1 Stall→fault:** ISENSE over limit for N samples ⇒ drive 0, driver asleep, motion
  inhibited; `nFault` pin also faults. HW: low `xl`, command a move into a stop.
- **O2 Clear:** `x` (and calibration) clears the latch.
- **O3 Soft limits (once built):** a normal shift never commands past gear-1/gear-11 ± margin.
- **O4 Plausibility (once built):** an implausibly far target ⇒ uncertainty flag, hold.

### Control quality (after tuning)
- **Q1 No overshoot/overshift on a step** (SIL assert already present).
- **Q2 Steady-state error within band** once Ki/Kd are set (currently Kp-only → standing error).

---

## Commit history (reverse chronological)

| Commit | Summary |
|--------|---------|
| (this) | add DEVLOG engineering log (session summary, regression analysis, safety design, test plan); wire into commit workflow |
| `f50ea34` | escape overcurrent fault by entering calibration (clear + hold) — HW verified |
| `0d64e2d` | boot target from gear table; fix reversed cal jog — **boot-slam regression (boot-target half)** |
| `1a068c8` | persist current gear on shift so reboot keeps the gear |
| `3fb07ed` | persist turn count on change; fix boot angle seed; document multi-turn risk |
| `62d54f0` | add debug monitor (`?`/`v`) and guided button calibration |
| `01fc7b6` | add CLAUDE.md and recover tuning defaults as comments |
| `8023035` | re-write based on new plan (PID/motor_sm/gears/console/telemetry/overcurrent/migration) |
| `ca07aee` | setup for automation (baseline before this effort) |
