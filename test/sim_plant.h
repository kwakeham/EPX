/**
 * @file sim_plant.h
 * @brief Minimal motor + derailleur plant model for off-target (host) testing.
 *
 * Second-order model: drive produces acceleration, opposed by viscous damping.
 *   accel = k_drive * drive - damping * vel
 *   vel  += accel * dt
 *   pos  += vel   * dt
 * Optionally clamps to hard end-stops. Position units are degrees, matching the
 * controller's angle units; drive is the same -400..400 the firmware produces.
 */

#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdbool.h>

typedef struct
{
    float pos;        // degrees
    float vel;        // degrees / second
    float k_drive;    // accel per unit drive
    float damping;    // viscous damping (1/s)
    float dt;         // step period (s)
    bool  use_stops;  // clamp to [stop_lo, stop_hi]
    float stop_lo, stop_hi;
} plant_t;

/** Initialise with the default motor/derailleur parameters at period dt. */
void plant_init(plant_t *p, float dt);

/** Advance one step with the given drive; returns the new position. */
float plant_step(plant_t *p, float drive);

#endif // SIM_PLANT_H
