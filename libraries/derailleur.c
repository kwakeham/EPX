/**
 * @file derailleur.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief The nominal cog-spacing profile. See derailleur.h.
 *
 * These are the measured EPS 11-speed relative derailleur positions (Shifting.md)
 * and capture the real non-linear cog spacing. Only the shape matters: the affine
 * fit (gears_fit_profile) scales+offsets this through the two captured calibration
 * references (gear 2 and gear 10), so absolute scale and offset are per-bike.
 */

#include "derailleur.h"

const float gear_profile_nominal[NUM_REAR_GEARS] =
{
    6272.936f, 5303.239f, 4530.330f, 3811.599f, 3079.866f, 2402.238f,
    1731.913f, 1137.348f,  533.973f,  -89.428f, -665.268f
};
