/**
 * @file derailleur.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief SDK-free "file description" of the derailleur: gear count, the nominal
 *        (non-linear) cog spacing profile, and the overshift/dwell value type.
 *
 * The actual per-bike numbers are computed by calibration (capture gear 2 and
 * gear 10, then affine-fit the nominal profile through them) and stored in
 * epx_configuration_t (titan_mem.h). This header just defines the shape and the
 * shared constants/types so both the firmware and the host SIL can use them.
 */

#ifndef DERAILLEUR_H
#define DERAILLEUR_H

#include <stdint.h>

#define NUM_REAR_GEARS   11   // EPS 11-speed
#define NUM_FRONT_POS    2    // small / big chainring
#define NUM_DIRS         2    // up / down
#define DIR_UP           0
#define DIR_DOWN         1
#define GEAR_REF_LO_IDX  1    // gear 2  (0-based index into gear_pos[])
#define GEAR_REF_HI_IDX  9    // gear 10

/**
 * @brief Overtravel applied when shifting into a gear, then held (dwell) before
 *        settling back to the gear's rest position.
 *        overshift == 0 means: no overtravel AND no dwell (skip entirely).
 *        Units: overshift in the same angle units as gear_pos; dwell in ms.
 */
typedef struct
{
    int16_t overshift;
    int16_t dwell_ms;
} overshift_t;

/**
 * @brief Nominal relative cog spacing for the rear cassette (shape only). The
 *        units are arbitrary; calibration scales+offsets this to land gear 2 and
 *        gear 10 on the captured positions. Edit these with measured EPS ratios;
 *        the default is even spacing (which reduces to linear interpolation).
 *
 *        PLACEHOLDER: the default is even spacing -- there are no historical
 *        numbers to recover (the real cassette was never characterised in
 *        source). The true EPS 11-speed cog spacing is non-linear and must be
 *        measured, then entered in gear_profile_nominal[] (see derailleur.c).
 */
extern const float gear_profile_nominal[NUM_REAR_GEARS];

#endif // DERAILLEUR_H
