# EPX — Project Overview

EPX is a wireless electronic shifting controller for a **Campagnolo EPS 11-speed derailleur**.
It runs bare-metal on an **nRF52832 + S132 SoftDevice** (nRF5 SDK 17), drives the derailleur
motor through a **DRV8874** H-bridge, and senses position with a **sin/cos hall sensor**.
Riders shift over buttons or BLE; a UART console + RTT logs are used for bring-up/tuning.

Build/flash/console specifics: [CLAUDE.md](CLAUDE.md). Running history + test plan:
[DEVLOG.md](DEVLOG.md).

---

## Position sensing & the relative-actuator design (the important part)

The motor/derailleur is a **relative actuator**: the sin/cos hall sensor is **absolute only
within one 360° magnet turn**. The derailleur's full travel spans several turns, so true
position is:

```
absolute_position = current_rotations * 360 + within_turn_angle
```

`within_turn_angle` (0–360°) comes from `atan2(sin, cos)` every sample. `current_rotations`
is incremented/decremented in `angle()` ([libraries/mpos.c](libraries/mpos.c)) each time the
within-turn angle wraps across the 180° boundary between 256 Hz samples. The turn count is
the only state that cannot be re-derived from the sensor after a power loss.

### How position survives power loss (the Campagnolo method)

This mirrors how Campagnolo EPS handles it and is considered the **lowest-risk approach for
an on-bike power loss**:

1. **Store the turn count (and current gear) to flash**, frequently and on major events
   (motor settle/idle, each turn-boundary crossing, each shift, BLE disconnect). Persistence
   is via FDS ([titan_mem.c](titan_mem.c)): records are **appended** into an over-provisioned
   region and a **page erase / garbage-collect runs only every so often**. Over-provision the
   pages so accumulated writes stay within flash endurance over the product lifetime.
2. **On boot / recovery:**
   - read the sensor for the **relative (within-turn) position**;
   - load **`current_rotations`** and **`current_gear`** from the stored position record;
   - compute `position = stored_rotations * 360 + sensor` ([mpos.c](libraries/mpos.c)
     `mpos_calculate_angle`);
   - set the **target from the stored gear**: `gear_pos[current_gear]` from the calibrated
     configuration ([data_handler.c](libraries/data_handler.c) `data_handler_set_boot_target`,
     called from [main.c](main.c)).

Because the derailleur **is not easily back-driven**, the physical position barely moves
while powered off — so `stored_rotations + sensor` reconstructs the true position to within a
few degrees, and the target (`gear_pos[current_gear]`) is essentially where the derailleur
already is. The on-boot move is therefore tiny in normal use.

### Notes / vestigial state / edge cases

- **`target_angle` (in the position record) is vestigial.** It is a remnant from when the
  code worked purely in angles. Position and target are reconstructed from **turns + gear**,
  so `target_angle` does not need to be stored. It can be removed in a future cleanup
  (struct change → flash migration); currently it is just ignored on boot when the gears are
  calibrated.
- **Rollover edge case (deferred):** if power is lost with the within-turn angle right at the
  180° wrap boundary, the restored turn count can be ambiguous by ±1 turn. This is a known,
  accepted edge case left for future work (e.g. hysteresis around the boundary, or biasing
  the stored value). It does not occur in the common case.
- **If the turn count is ever wrong** (e.g. heavy bench handling, or recalibrating gears
  against a different turn baseline), the boot target can be far from the physical position.
  Recovery is a deliberate **recalibration on a stand** (console `c`, jog with buttons,
  capture gear 2 & 10). Entering calibration also clears a latched overcurrent fault and
  holds position, so you can always escape an end-of-travel stall. **No autonomous homing**
  to a hard stop is ever performed (a reboot mid-ride must never fling the derailleur).

---

## Control loop (brief)

256 Hz cooperative loop ([mpos.c](libraries/mpos.c) `mpos_motor_drive`):
- **PID** ([PID_controller.c](libraries/PID_controller.c)) `pid_ctrl_t`: anti-windup,
  derivative-on-measurement, dt-scaled gains. Gains persisted (`p`/`i`/`d`).
- **Motor sleep** ([motor_sm.c](libraries/motor_sm.c)): explicit HOLDING/MOVING state machine;
  driver sleeps once settled in band.
- **Overshift/dwell** ([shift_seq.c](libraries/shift_seq.c)): optional overtravel → dwell →
  settle per gear / front position / direction.
- **Overcurrent fault**: ISENSE (AIN1) over a configurable limit, or the DRV8874 `nFault`
  pin, latches a fault that stops drive and inhibits motion until cleared (`x` or calibration).

## Gears & calibration (brief)

Capture **gear 2** and **gear 10** (`g l` / `g h`, or guided `c` + buttons), then affine-fit
an editable nominal cog profile ([gears.c](libraries/gears.c), [derailleur.h](libraries/derailleur.h))
to fill all 11 positions (`g i`). Gear table, overshift/dwell, gains, ISENSE limits, and the
position record all persist to flash; tuning lives in flash, not source.

## Interfaces (brief)

- **Buttons:** Btn1 = up, Btn2 = down, Btn3 = mode (long-press: calibration).
- **Console (UART 115200 on the J-Link CDC port):** commands echo replies; `?` one-shot
  status, `v<n>` low-rate monitor, `y<n>` CSV telemetry. `NRF_LOG` is on RTT.
- **BLE:** Nordic UART Service (same command grammar) + buttonless DFU.
