/**
 * @file test_loop.c
 * @brief Host SIL harness: runs the REAL control logic (PID + sleep state
 *        machine + gear interpolation) against a simulated plant, with no SDK.
 *
 * Two jobs:
 *   1. Unit asserts (gear interpolation spacing; no-overshoot step response).
 *   2. CSV time-series to stdout, same columns the firmware streams over RTT
 *      (telemetry.c), so a host plot and a hardware capture line up directly.
 *
 * Build:  make -C test
 * Run:    ./test/sil > run.csv      (asserts print to stderr, CSV to stdout)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "PID_controller.h"
#include "motor_sm.h"
#include "gears.h"
#include "sim_plant.h"

#define DT            (1.0f / 256.0f)
#define ANGLE_BAND    10.0f
#define SETTLE_TICKS  64
#define MAX_TICKS     2048   /* ~8 s per move */

/* Default gains for the sim_plant model above. These are NOT the hardware gains
 * (the real motor/derailleur differs) -- tune against the real plant, but this
 * shows an overdamped, non-overshooting starting point. */
static float kp = 8.0f;
static float ki = 0.5f;
static float kd = 3.0f;

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { fprintf(stderr, "FAIL: %s\n", msg); g_fail = 1; } \
    else         { fprintf(stderr, "PASS: %s\n", msg); } } while (0)

/* Run one move from `start` to `target`. Streams CSV when print != 0.
 * Returns the worst overshoot beyond target (0 if none). t0 offsets the time column. */
static float run_move(float target, float start, int print, int t0, int *t_end)
{
    pid_ctrl_t pid;
    pid_init(&pid, &kp, &ki, &kd, -400.0f, 400.0f, -2000.0f, 2000.0f);

    motor_sm_t sm;
    motor_sm_init(&sm, ANGLE_BAND, SETTLE_TICKS);

    plant_t plant;
    plant_init(&plant, DT);
    plant.pos = start;
    pid_reset(&pid, start);

    float dir = (target >= start) ? 1.0f : -1.0f;
    float overshoot = 0.0f;
    int t;

    for (t = 0; t < MAX_TICKS; t++)
    {
        float current = plant.pos;

        float drive = 0.0f;
        if (sm.state == MOTOR_MOVING)
            drive = pid_update(&pid, target, current, DT);

        motor_action_t act;
        int driving = motor_sm_step(&sm, target, current, &act);
        if (!driving) drive = 0.0f;
        if (act.pid_reset) pid_reset(&pid, current);

        plant_step(&plant, drive);

        float past = dir * (plant.pos - target);     /* >0 means past the target */
        if (past > overshoot) overshoot = past;

        if (print)
            printf("%d,%.1f,%.2f,%.2f,%.1f,%.2f,%d\n",
                   (t0 + t) * 1000 / 256, target, current,
                   target - current, drive, pid.integral, (int)sm.state);

        if (sm.state == MOTOR_HOLDING) { t++; break; }
    }
    if (t_end) *t_end = t0 + t;
    return overshoot;
}

static void test_gears_interpolate(void)
{
    int32_t gp[11] = {0};
    /* Capture gear 2 (idx 1) at 100 and gear 10 (idx 9) at 900 -> spacing 100. */
    int ok = gears_interpolate(gp, 11, 1, 100, 9, 900);
    CHECK(ok, "interpolation runs with valid args");
    CHECK(gp[1] == 100, "gear 2 == low ref");
    CHECK(gp[9] == 900, "gear 10 == high ref");
    CHECK(gp[0] == 0,   "gear 1 extrapolated below");
    CHECK(gp[5] == 500, "gear 6 evenly spaced");
    CHECK(gp[10] == 1000, "gear 11 extrapolated above");

    int bad = gears_interpolate(gp, 11, 3, 100, 3, 900);
    CHECK(!bad, "rejects equal indices");
}

static void test_no_overshoot(void)
{
    float os_up   = run_move(100.0f, 0.0f, 0, 0, NULL);
    float os_down = run_move(0.0f, 100.0f, 0, 0, NULL);
    fprintf(stderr, "  overshoot up=%.2f down=%.2f deg\n", os_up, os_down);
    CHECK(os_up   < ANGLE_BAND / 2.0f, "step up does not overshoot");
    CHECK(os_down < ANGLE_BAND / 2.0f, "step down does not overshoot");
}

int main(void)
{
    fprintf(stderr, "== EPX host SIL ==\n");
    test_gears_interpolate();
    test_no_overshoot();

    /* Emit a multi-move CSV to stdout for plotting. */
    printf("t_ms,target,current,error,drive,integral,state\n");
    int t = 0;
    run_move(100.0f,  0.0f, 1, t, &t);
    run_move(60.0f, 100.0f, 1, t, &t);
    run_move(160.0f, 60.0f, 1, t, &t);

    fprintf(stderr, g_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return g_fail;
}
