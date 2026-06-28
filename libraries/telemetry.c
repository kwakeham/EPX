/**
 * @file telemetry.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief CSV time-series telemetry over SEGGER RTT up-channel 1. See telemetry.h.
 */

#include "telemetry.h"
#include <stdio.h>
#include <string.h>
#include "SEGGER_RTT.h"

#define TELEMETRY_RTT_CHANNEL 1

static bool     m_enabled = false;
static uint32_t m_tick    = 0;
static char     m_line[128];
static uint8_t  m_up_buffer[1024];

void telemetry_init(void)
{
    SEGGER_RTT_ConfigUpBuffer(TELEMETRY_RTT_CHANNEL, "telemetry",
                              m_up_buffer, sizeof(m_up_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

void telemetry_set_enabled(bool enabled)
{
    m_enabled = enabled;
    m_tick    = 0;
    if (enabled)
    {
        static const char header[] = "t_ms,target,current,error,drive,integral,state\n";
        SEGGER_RTT_Write(TELEMETRY_RTT_CHANNEL, header, sizeof(header) - 1);
    }
}

bool telemetry_is_enabled(void)
{
    return m_enabled;
}

void telemetry_tick(float target, float current, float drive, float integral, int state)
{
    if (!m_enabled) return;

    uint32_t t_ms = (m_tick * 1000u) / 256u;
    m_tick++;

    int n = snprintf(m_line, sizeof(m_line),
                     "%lu,%ld,%ld,%ld,%ld,%ld,%d\n",
                     (unsigned long)t_ms,
                     (long)(target * 100.0f),
                     (long)(current * 100.0f),
                     (long)((target - current) * 100.0f),
                     (long)drive,
                     (long)(integral * 100.0f),
                     state);
    if (n > 0)
    {
        SEGGER_RTT_Write(TELEMETRY_RTT_CHANNEL, m_line, (unsigned)n);
    }
}
