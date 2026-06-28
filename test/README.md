# EPX host SIL (software-in-the-loop) harness

Runs the **real** control logic — `PID_controller.c`, `motor_sm.c`, `gears.c`
from `../libraries` — against a simulated motor/derailleur (`sim_plant.c`) on the
PC, with no nRF SDK. Lets you tune PID gains and verify no overshoot/overshift
before touching hardware, then confirm the same CSV shape on-target over RTT.

## Build & run

```sh
make -C test          # builds ./test/sil
make -C test run      # runs it: asserts -> stderr, CSV time-series -> run.csv
# or:
./test/sil > run.csv
```

Exit code is non-zero if any assert fails (CI-friendly).

## Output

- **Unit asserts** (stderr): gear-interpolation spacing, and a no-overshoot check
  on a simulated step response.
- **CSV** (stdout): `t_ms,target,current,error,drive,integral,state` — identical
  columns to the firmware telemetry stream (`libraries/telemetry.c`). Plot it
  (e.g. `gnuplot`, pandas) to inspect the response.

## Tuning

`kp/ki/kd` in `test_loop.c` and the plant params in `sim_plant.c` (`k_drive`,
`damping`) are the knobs. These gains are tuned to the *sim* model, not the real
hardware — adjust the plant params to match the measured motor response, then the
gains you find here are a sane starting point for the firmware (`p`/`i`/`d`
commands).
