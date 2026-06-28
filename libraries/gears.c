/**
 * @file gears.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief SDK-free gear-position interpolation. See gears.h.
 */

#include "gears.h"

static int32_t lround_div(float v)
{
    return (int32_t)(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
}

bool gears_interpolate(int32_t *gear_pos, int n,
                       int idx_lo, int32_t pos_lo,
                       int idx_hi, int32_t pos_hi)
{
    if (gear_pos == 0 || n <= 0) return false;
    if (idx_lo < 0 || idx_hi < 0 || idx_lo >= n || idx_hi >= n) return false;
    if (idx_hi == idx_lo) return false;

    float spacing = (float)(pos_hi - pos_lo) / (float)(idx_hi - idx_lo);
    for (int i = 0; i < n; i++)
    {
        gear_pos[i] = pos_lo + lround_div(spacing * (float)(i - idx_lo));
    }
    return true;
}
