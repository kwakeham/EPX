# EPX — Claude working notes

EPX is a wireless motor controller for a **Campagnolo EPS 11-speed derailleur**, bare-metal on
**nRF52832 + S132 SoftDevice, nRF5 SDK 17.0.2**. It drives a DRV8874 H-bridge to move the derailleur
to discrete gear angles, using a sin/cos hall potentiometer (SAADC) for position feedback. The control
loop runs cooperatively at 256 Hz in [main.c](main.c).

**System design + the position/turns/recovery model: [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md).**
Running history + test plan: [DEVLOG.md](DEVLOG.md).

## Layout
- [libraries/](libraries/) — application logic:
  - [mpos.c](libraries/mpos.c) control loop: reads angle, runs PID, sleep state machine, overcurrent, telemetry.
  - [PID_controller.c](libraries/PID_controller.c) reusable `pid_ctrl_t` (anti-windup, deriv-on-measurement, dt-scaled).
  - [motor_sm.c](libraries/motor_sm.c) HOLDING/MOVING driver-sleep state machine.
  - [shift_seq.c](libraries/shift_seq.c) overshift→dwell→settle sequencer.
  - [gears.c](libraries/gears.c) gear-position interpolation / profile fit; [derailleur.h](libraries/derailleur.h) gear/overshift data model.
  - [data_handler.c](libraries/data_handler.c) console/BLE command parser; [console.c](libraries/console.c) / [telemetry.c](libraries/telemetry.c) UART I/O.
- [drivers/](libraries/) — [drv8874.c](drivers/drv8874.c) motor driver.
- [titan_mem.c](titan_mem.c) — FDS flash persistence (config + position records, versioned migration).
- [ble_cus.c](ble_cus.c) — BLE stack, NUS, DFU, and the `app_uart` console UART.
- [pca10040/s132/armgcc/](pca10040/s132/armgcc/) — Makefile; [pca10040/s132/config/sdk_config.h](pca10040/s132/config/sdk_config.h).
- [test/](test/) — host SIL harness (pure logic vs a simulated plant).
- SDK lives at `../nRF5_SDK_17.0.2_d674dde` (referenced by the Makefile).

## Build / flash / talk
- **Build:** `make` in [pca10040/s132/armgcc](pca10040/s132/armgcc). Toolchain: arm-none-eabi-gcc, GnuWin32 make.
- **Flash:** `make flash_dfu` — **not** `make flash`. A buttonless-DFU bootloader is present
  (UICR→0x78000); it refuses to launch an app flashed without matching settings, so plain `make flash`
  leaves the device idling in the bootloader (silent console, no app peripherals). `flash_dfu`
  generates + merges bootloader settings (modular `nrfutil nrf5sdk-tools`, settings version 2).
- **Console:** UART **115200 8N1** on the J-Link CDC UART COM port. The Windows COM number churns on
  re-enumeration (seen as COM5/7, COM9/10) — probe and pick the one that replies. Drive it from
  PowerShell `System.IO.Ports.SerialPort` (no extra tooling). Commands echo replies; `y4` starts CSV
  telemetry (`y0` off). Command grammar lives in `data_handler_command_processor`.
- **Logs:** `NRF_LOG` is on **RTT** (separate from the console UART). Read with JLinkRTTLogger; pass
  the control-block address from `arm-none-eabi-nm _build/*.out | grep _SEGGER_RTT` if auto-search fails.
- **Host tests:** `make -C test` (needs a host C compiler — not present on the bench machine).

## Tuning lives in flash, not source
PID gains (`p`/`i`/`d`), gear positions (calibrate with `g l` / `g h` / `g i`), front position (`b`),
and overshift/dwell (`o`) are set over the console and persisted to flash by [titan_mem.c](titan_mem.c).
They are **not** hard-coded in source and are reset to safe defaults when `CONFIG_VERSION` is bumped.
After a flash wipe / version bump, re-run calibration and re-tune. See the recovered tuning references
commented in [libraries/mpos.c](libraries/mpos.c) and [titan_mem.c](titan_mem.c) for starting points.

## Multi-turn position (read before touching position code)
The sin/cos hall sensor is **absolute only within one 360° turn**; true position is
`current_rotations * 360 + within-turn angle`. `current_rotations` (in
`epx_position_configuration_t`, counted in `angle()` in [libraries/mpos.c](libraries/mpos.c)) is the
only state that can't be re-derived from the sensor after power loss.

The handling is the **store-turns-to-flash (Campagnolo) method** — full design and rationale in
[PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md). In short: persist `current_rotations` + `current_gear`
often (settle/idle, each turn crossing, each shift, disconnect); on boot reconstruct
`position = stored_rotations·360 + sensor` and set the target from `gear_pos[current_gear]`. It's
low-risk because the derailleur isn't easily back-driven, so a power cut loses only a few degrees.

Invariants to preserve when editing position code:
- `current_rotations` + `current_gear` are the persisted source of truth; `target_angle` is vestigial
  (don't rely on it; it's a removal candidate).
- Seed `angle_old` from the first real reading (not `target_angle`) to avoid a first-sample miscount.
- **No autonomous homing to a hard stop** — a reboot mid-ride must never fling the derailleur.
  Recovery from a bad turn count is deliberate recalibration (`c`), which also clears faults + holds.
- Known deferred edge case: ±1-turn ambiguity if power is lost right at the 180° wrap.

## Commit workflow
After each meaningful, **tested** unit of work, make a focused commit and **push to origin/master**:
- Terse, imperative messages matching the existing history (e.g. "extract drive strength to a function").
- Don't commit build artifacts — `.gitignore` already excludes `_build/` and the DFU settings hex.
- End every commit message with:
  `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`
- **Add a one-line entry to the commit-history table in [DEVLOG.md](DEVLOG.md)** and, when behavior or
  design changes, update the relevant DEVLOG section. DEVLOG is the narrative history + open design
  threads + test plan; keep it current so a fresh session can follow the work.

"Meaningful + tested" is a judgment call (it built, and where possible ran on hardware), so this is a
working agreement here rather than an automated hook.
