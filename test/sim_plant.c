/**
 * @file sim_plant.c
 * @brief Minimal motor + derailleur plant model. See sim_plant.h.
 */

#include "sim_plant.h"

void plant_init(plant_t *p, float dt)
{
    p->pos       = 0.0f;
    p->vel       = 0.0f;
    p->k_drive   = 2.0f;    // tune to match the real motor's responsiveness
    p->damping   = 10.0f;
    p->dt        = dt;
    p->use_stops = false;
    p->stop_lo   = 0.0f;
    p->stop_hi   = 0.0f;
}

float plant_step(plant_t *p, float drive)
{
    float accel = p->k_drive * drive - p->damping * p->vel;
    p->vel += accel * p->dt;
    p->pos += p->vel * p->dt;

    if (p->use_stops)
    {
        if (p->pos < p->stop_lo) { p->pos = p->stop_lo; if (p->vel < 0.0f) p->vel = 0.0f; }
        if (p->pos > p->stop_hi) { p->pos = p->stop_hi; if (p->vel > 0.0f) p->vel = 0.0f; }
    }
    return p->pos;
}
