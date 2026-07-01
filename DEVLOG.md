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

## Overshift format + calibration ergonomics (2026-06-29)

Captured the real shifting behaviour from `Shifting.md` and fixed two bench ergonomics issues.

**Overshift is now per-mille of the shift distance (was an absolute angle).** The storage shape
was already right — `rear_overshift[gear][front][dir]` keyed by the destination gear, which for
single-step shifts *is* the transition — so only the units and defaults changed:
- `overshift_t.overshift` → `overshift_pm` ([derailleur.h](libraries/derailleur.h)): per-mille of
  the shift's gear-to-gear distance.
- Shift compute ([data_handler.c](libraries/data_handler.c) `data_handler_shift_gear_handler`):
  `signed_overshift = overshift_pm * (final_pos - prev_pos) / 1000`. The signed span gives the
  direction automatically, so the old `(final>=prev)?+:-` branch is gone. This makes the overshoot
  scale with the calibrated spacing and survive recalibration.
- `o` command now takes/lists per-mille: `o <gear> <front> <dir> <permille> <dwell_ms>`.
- **Seeded defaults** from the measured EPS table ([titan_mem.c](titan_mem.c) `EPX_CONFIG_DEFAULTS`),
  dwell 1200 ms. The table is direction-**asymmetric**: upshifts into g7–g11 need no overshift while
  the matching downshifts do. `CONFIG_VERSION 2 → 3` (resets the **config** to these defaults →
  one recalibration needed; the **position record / turn count is a separate, unversioned record and
  is preserved**).

**Gear profile populated** ([derailleur.c](libraries/derailleur.c)): `gear_profile_nominal[]` now
holds the measured non-linear EPS spacing instead of the even-spacing placeholder, so the 9
interpolated gears land more accurately (calibration still affine-fits via captured gear 2 & 10).

**Calibration jog ~3× faster** ([data_handler.c](libraries/data_handler.c)): `CALIB_JOG_FINE` 5→15,
`CALIB_JOG_COARSE` 30→90 (~450°/s while held; ~90° coast after release).

**Btn3 calibration entry now needs a deliberate ~2 s hold** ([data_handler.c](libraries/data_handler.c)):
`LONGPRESS_INTERVAL_MS` (200 ms) is shared by all buttons, so instead of slowing it, calibration
entry counts `CALIB_ENTER_LONG_TICKS` (10) consecutive `CH3_LONG` events and resets on `CH3_RELEASE`.
Btn1/Btn2 jog/shift long-press timing is unchanged.

**Memory-format investigation (requested):** all four intended turn-count save triggers already exist
(settle, turn-crossing, shift, BLE disconnect) — no missing trigger. This feature only touches the
**config** record; the turn count lives in the unversioned position record and is untouched. Noted
pre-existing gaps (not fixed here): the position record has no version guard, and saves are async
(power-loss write window) — both already tracked as deferred items below.

**Status:** builds clean (arm-none-eabi-gcc, no warnings). **Not yet flashed/HW-verified** — next:
`make flash_dfu`, recalibrate once (config reset), confirm turn count survived, and exercise the
overshift table per the test plan.

---

## Boot-slam analysis (#1) — corrected: NOT a code regression

**Symptom:** on boot the derailleur drove hard into an end-of-travel stop and latched an
overcurrent fault (observed: `tgt -10708` while physically at `pos -9259`).

**Corrected root cause (per domain input, Campagnolo veteran):** the boot behavior is the
**intended design**, not a bug. `data_handler_set_boot_target()` (`0d64e2d`,
[data_handler.c](libraries/data_handler.c), called at [main.c:162](main.c#L162)) sets the
target from the stored gear — `gear_pos[current_gear]` — which is exactly the Campagnolo
recovery method (see [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)). It is safe **because the
derailleur is not easily back-driven**, so `stored_rotations·360 + sensor` reconstructs the
real position to within a few degrees and the gear target ≈ where it already is.

The slam happened only because our **bench state had an inconsistent turn count**: heavy
manual handling, many resets, Kp=1 driving to assorted targets, and recalibrating gears
against a shifted turn baseline left `current_rotations` out of sync with the calibrated
`gear_pos[]`. In normal on-bike use this does not occur. **Recovery is recalibration**
(console `c` + button jog/capture), which also clears the fault and holds position.

**No reversion needed.** Earlier this log proposed reverting the boot-target-from-gear and a
"boot = hold, not seek" change — **that was wrong** and is withdrawn. Keep the stored-turns +
gear-target design.

**Keep (genuine fixes/improvements):**
- `3fb07ed` `angle_old` prime-from-first-reading: the older `angle_old = target_angle % 360`
  had a latent bug — a *negative* saved target (e.g. -7359 % 360 = -159) vs a 0–360 reading
  trips a false 180° wrap and miscounts a turn on the first sample. The prime fix removes that.
- `1a068c8` persist-gear-on-shift and `3fb07ed` save-on-rotation-change: additive, more
  frequent turn-count/gear persistence — aligned with the store-turns-often method.

**`target_angle` is vestigial** (remnant of the angle-only era); position+target come from
turns+gear. Candidate for removal in a future cleanup (struct change → flash migration).

---

## Position-loss handling — RESOLVED: store-turns (Campagnolo method)

The accepted approach is the **store-turns-to-flash** method (implemented; see
[PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)): persist `current_rotations` + `current_gear`
frequently; on boot reconstruct `position = stored_rotations·360 + sensor` and target from
`gear_pos[current_gear]`. It is the lowest-risk method for on-bike power loss because the
derailleur is not easily back-driven (a power cut loses only a few degrees).

**Settled decisions:**
- **No autonomous homing** to a hard stop, ever (a reboot mid-ride must not fling the
  derailleur). This stays vetoed.
- Recovery from an inconsistent turn count = **deliberate recalibration on a stand**; the
  `c` calibration entry also clears a latched fault and holds position (escape hatch).
- The earlier "boot = hold / soft-limits / uncertainty-flag" proposal is **not** the
  direction — deferring to the store-turns design.

**Deferred (future, low priority):**
- **180° rollover edge case:** ±1-turn ambiguity if power is lost exactly at the wrap
  boundary. Future mitigation (boundary hysteresis / bias) — not common.
- Optional hardening only if ever wanted: a power-fail-comparator last-gasp save and/or a
  hold-up cap to further shrink the loss window. Needs the supply/hold-up details.
- `target_angle` cleanup (remove the vestigial field; struct change → flash migration).

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

### Startup / boot (store-turns recovery)
- **S1 Reconstruct + tiny move:** with a *consistent* stored turn count, boot computes
  `pos = stored_rotations·360 + sensor` and targets `gear_pos[current_gear]`; since the
  derailleur wasn't back-driven, the on-boot move is small. HW: settle, power-cycle, `?`
  shows tgt≈pos, small err.
- **S2 Inconsistent turns → recalibration:** if the stored turns are wrong, boot can target
  far off (and may fault into a stop). The accepted recovery is `c` recalibration — verify
  it restores S1. (This is the bench state we hit; not a normal on-bike case.)
- **S3 First-sample no miscount:** boot doesn't spuriously ±360 the turn count, incl. a
  negative saved target. HW: `?` pos stable across reset (verified for `3fb07ed`).

### Power loss / random reboot
- **P1 Reboot at rest:** position + gear exactly retained (saved on settle/idle).
- **P2 Reboot mid-move (no turn crossing):** within-turn angle read fresh; small error.
- **P3 Reboot mid-move across a turn boundary:** turn count saved on crossing keeps absolute
  position within one save-window; quantify worst-case error.
- **P4 Rollover edge case (deferred):** power lost with the within-turn angle at the 180°
  wrap ⇒ ±1-turn ambiguity. Document/measure; mitigation is future work.

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

### Overcurrent / end-stop
- **O1 Stall→fault:** ISENSE over limit for N samples ⇒ drive 0, driver asleep, motion
  inhibited; `nFault` pin also faults. HW: low `xl`, command a move into a stop.
- **O2 Clear:** `x` (and calibration) clears the latch.

### Control quality (after tuning)
- **Q1 No overshoot/overshift on a step** (SIL assert already present).
- **Q2 Steady-state error within band** once Ki/Kd are set (currently Kp-only → standing error).

---

## HIL bench harness (`tools/epx_hil/`)

Python (`pyserial`) hardware-in-the-loop harness that drives the console and turns
the plan above into repeatable, logged runs. It is the bench-side complement to the
host SIL in `test/` (SIL = pure logic vs a sim plant; HIL = real firmware on real
hardware). Run `python -m epx_hil <cmd>` from `tools/`; `README.md` has the details.

- **Reads the firmware's own instrumentation:** one UART carries command replies,
  the `y` CSV telemetry, and the `e` `#event` HIL log; the reader thread demuxes by
  line shape. `#boot/#shift/#turn/#save/#cal/#fault` are ground-truth cross-checks.
- **Per-move metrics** (overshoot vs the *final* gear position, settle time from the
  MOV→HLD edge, steady-state error, turns vs `#turn` count, peak/hold ISENSE) →
  `summary.md/.csv` under a never-overwritten `logs/<date>/<time>_<label>/` folder.
- **Two-point calibration** is automated with bounded relative jogs (safety cap +
  stall/fault abort), mirroring the `gl`/`gh`/`gi` console path, then `s9` to seat
  gear 10 without a fling — automates **C1**, and the fault-escape path around **C3**.
- **Coverage of the plan:** far/sweep moves + repeatability (**H1/H2**, control
  quality **Q1/Q2**); `test safety` = overcurrent latch/clear (**O1/O2**), reboot
  mid-hold recovery (**P1–P3**, **S1**), boot-slam (no fling); `test tuning` =
  step-response/PID, hold current, loop-timing jitter; `test endurance` = g1↔g11
  cycling + flash-persistence stress.
- **Offline-testable:** parsers + metric math run with no hardware (`pytest epx_hil`);
  old logs can be re-scored via `epx_hil.offline`.

Open follow-ups: **P4** (180° rollover ±1 turn) needs a precise power-cut trigger the
harness can't yet force; telemetry `current` is absolute angle but there is no explicit
`rot` column (reconstructed as `floor(current/360)` — add a column if a case needs it).

---

## Commit history (reverse chronological)

| Commit | Summary |
|--------|---------|
| (this) | add HIL test battery (`tools/epx_hil/tests/`): core-motion (sweep+repeatability), safety (overcurrent/power-loss/boot-slam), tuning (step/hold/jitter), endurance+flash; registry + `add_check` summary rows |
| `3e4ab4d` | add Python HIL harness (`tools/epx_hil/`): serial demux, autodetect, two-point calibration FSM, telemetry capture + per-move metrics, timestamped logs, CLI; offline pytest |
| `0e06455` | add console `r` reboot command (deferred reset flushes pending flash + reply; emits `#boot` on restart) for the HIL harness |
| `b0061a1` | store overshift as per-mille of shift distance; seed EPS overshift table + gear profile; faster cal jog; ~2 s Btn3 hold to enter cal (CONFIG_VERSION 3) |
| `306027b` | add PROJECT_OVERVIEW.md (store-turns position design); correct boot-slam analysis (not a regression) |
| `dbad51f` | add DEVLOG engineering log (session summary, analysis, test plan); wire into commit workflow |
| `f50ea34` | escape overcurrent fault by entering calibration (clear + hold) — HW verified |
| `0d64e2d` | boot target from stored gear (Campagnolo recovery method); fix reversed cal jog |
| `1a068c8` | persist current gear on shift so reboot keeps the gear |
| `3fb07ed` | persist turn count on change; fix boot angle seed; document multi-turn risk |
| `62d54f0` | add debug monitor (`?`/`v`) and guided button calibration |
| `01fc7b6` | add CLAUDE.md and recover tuning defaults as comments |
| `8023035` | re-write based on new plan (PID/motor_sm/gears/console/telemetry/overcurrent/migration) |
| `ca07aee` | setup for automation (baseline before this effort) |
