/**
 * @file gears.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief SDK-free gear-position interpolation.
 *
 * Rather than entering every gear angle by hand, two reference gears are
 * captured (e.g. gear 2 and gear 10) and the rest are linearly interpolated
 * from the constant spacing between them. Even spacing is what keeps shifts
 * from over/under-shooting a gear.
 */

#ifndef GEARS_H
#define GEARS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Fill gear_pos[0..n-1] by linear interpolation through two references.
 *
 * spacing = (pos_hi - pos_lo) / (idx_hi - idx_lo); gear_pos[i] = pos_lo +
 * (i - idx_lo) * spacing, computed with rounding. Indices are 0-based.
 *
 * @return false (no change) if arguments are out of range or idx_hi == idx_lo.
 */
bool gears_interpolate(int32_t *gear_pos, int n,
                       int idx_lo, int32_t pos_lo,
                       int idx_hi, int32_t pos_hi);

/**
 * @brief Fill gear_pos[0..n-1] by affine-fitting a nominal spacing profile so
 *        that gear idx_lo lands on pos_lo and gear idx_hi on pos_hi.
 *
 * scale  = (pos_hi - pos_lo) / (profile[idx_hi] - profile[idx_lo])
 * offset = pos_lo - scale * profile[idx_lo]
 * gear_pos[i] = round(offset + scale * profile[i])
 *
 * With an even profile this reduces to gears_interpolate(); a non-even profile
 * captures the cassette's non-linear cog spacing from just two captured points.
 *
 * @param profile array of at least n nominal positions (shape only).
 * @return false if args are out of range or profile[idx_hi]==profile[idx_lo].
 */
bool gears_fit_profile(int32_t *gear_pos, int n, const float *profile,
                       int idx_lo, int32_t pos_lo,
                       int idx_hi, int32_t pos_hi);

#endif // GEARS_H
