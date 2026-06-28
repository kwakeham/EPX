/**
 * @file derailleur.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief The nominal cog-spacing profile. See derailleur.h.
 *
 * Default is even spacing (0..10), which makes the affine fit equivalent to
 * linear interpolation. Replace these with the measured EPS 11-speed relative
 * derailleur positions to capture the real non-linear cog spacing; only the
 * shape matters (calibration handles absolute scale and offset).
 */

#include "derailleur.h"

const float gear_profile_nominal[NUM_REAR_GEARS] =
{
    0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f
};
