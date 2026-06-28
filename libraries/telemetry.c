/**
 * @file telemetry.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief CSV time-series telemetry over the UART. See telemetry.h.
 */

#include "telemetry.h"
#include "serial_io.h"
#include <stdio.h>
#include <string.h>

#define TELEMETRY_DEFAULT_DIV 4   // ~64 Hz at a 256 Hz loop

static uint16_t m_divider = 0;    // 0 = disabled
static uint16_t m_phase   = 0;
static uint32_t m_tick    = 0;
static char     m_line[128];

void telemetry_init(void)
{
    m_divider = 0;
    m_phase   = 0;
    m_tick    = 0;
}

void telemetry_set_rate(uint16_t divider)
{
    m_divider = divider;
    m_phase   = 0;
    m_tick    = 0;
    if (divider > 0)
    {
        static const char header[] = "t_ms,target,current,error,drive,integral,state,isense,fault\n";
        serial_write((const uint8_t *)header, sizeof(header) - 1);
    }
}

void telemetry_set_enabled(bool enabled)
{
    telemetry_set_rate(enabled ? TELEMETRY_DEFAULT_DIV : 0);
}

bool telemetry_is_enabled(void)
{
    return m_divider > 0;
}

void telemetry_tick(float target, float current, float drive, float integral,
                    int state, int16_t isense, bool fault)
{
    if (m_divider == 0) return;

    uint32_t t_ms = (m_tick * 1000u) / 256u;
    m_tick++;

    if (++m_phase < m_divider) return;
    m_phase = 0;

    int n = snprintf(m_line, sizeof(m_line),
                     "%lu,%ld,%ld,%ld,%ld,%ld,%d,%d,%d\n",
                     (unsigned long)t_ms,
                     (long)(target * 100.0f),
                     (long)(current * 100.0f),
                     (long)((target - current) * 100.0f),
                     (long)drive,
                     (long)(integral * 100.0f),
                     state,
                     (int)isense,
                     fault ? 1 : 0);
    if (n > 0)
    {
        serial_write((const uint8_t *)m_line, (uint16_t)n);
    }
}
