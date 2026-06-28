/**
 * @file console.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Interactive serial console over SEGGER RTT. See console.h.
 */

#include "console.h"
#include <string.h>
#include "SEGGER_RTT.h"
#include "data_handler.h"

#define CONSOLE_RTT_CHANNEL 0   // shares the terminal channel with NRF_LOG
#define CONSOLE_LINE_MAX    64

static char    m_line[CONSOLE_LINE_MAX];
static uint8_t m_len = 0;

void console_init(void)
{
    static const char banner[] =
        "\r\nEPX console ready. Commands: s<g> g l/h/i k y1/y0 ...\r\n";
    SEGGER_RTT_Write(CONSOLE_RTT_CHANNEL, banner, sizeof(banner) - 1);
}

void console_print(const uint8_t *data, uint16_t len)
{
    SEGGER_RTT_Write(CONSOLE_RTT_CHANNEL, data, len);
    SEGGER_RTT_Write(CONSOLE_RTT_CHANNEL, "\r\n", 2);
}

void console_poll(void)
{
    while (SEGGER_RTT_HasKey())
    {
        int c = SEGGER_RTT_GetKey();
        if (c < 0) break;

        if (c == '\r' || c == '\n')
        {
            if (m_len > 0)
            {
                data_handler_command(m_line, m_len);
                m_len = 0;
            }
        }
        else if (m_len < CONSOLE_LINE_MAX - 1)
        {
            m_line[m_len++] = (char)c;
        }
        // else: line overflow, drop the byte until a terminator resets m_len
    }
}
