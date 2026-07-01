/**
 * @file evt_log.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Tagged lifecycle-event output for HIL testing. See evt_log.h.
 */

#include "evt_log.h"
#include "serial_io.h"
#include <stdio.h>
#include <stdarg.h>

static uint32_t m_mask = 0;

void evt_log_init(void)
{
    m_mask = EVT_LOG_DEFAULT_MASK;
}

void evt_log_set_mask(uint32_t mask)
{
    m_mask = mask;
}

uint32_t evt_log_get_mask(void)
{
    return m_mask;
}

void evt_log(uint32_t cat, const char *fmt, ...)
{
    if ((m_mask & cat) == 0) return;

    char line[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 2, fmt, ap); // leave room for "\r\n"
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(line) - 3) n = (int)sizeof(line) - 3; // truncated
    line[n++] = '\r';
    line[n++] = '\n';
    // Non-blocking: events fire from the control loop and the button ISR, so a
    // blocking write could stall real-time code. Drop the line if the FIFO is full.
    serial_try_write((const uint8_t *)line, (uint16_t)n);
}
