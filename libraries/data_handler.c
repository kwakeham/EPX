/**
 * Copyright (c) 2019 - 2022, TITAN LAB INC
 *
 * All rights reserved.
 *
 *
 */
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include "data_handler.h"
#include "boards.h"
#include "math.h"
#include "ble_cus.h"
#include <string.h>
#include "titan_mem.h"
#include "mpos.h"
#include "gears.h"
#include "telemetry.h"
#include "evt_log.h"

#define NRF_LOG_MODULE_NAME datahandler
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


#define CMD_LENGTH 48     // command line buffer (multi-arg commands like 'o')
#define DATAOUTENABLE

// Two-point gear calibration reference indices come from derailleur.h
// (GEAR_REF_LO_IDX / GEAR_REF_HI_IDX), pulled in via titan_mem.h.

static int32_t ref_lo = 0, ref_hi = 0;
static bool    ref_lo_set = false, ref_hi_set = false;

// Guided calibration: jog to gear 2 / gear 10 with the buttons, capture with btn3.
#define CALIB_JOG_FINE   15   // degrees per button tap
#define CALIB_JOG_COARSE 90   // degrees per repeated long-press tick (~450 deg/s while held)
static bool calibrating = false;
static int  calib_step  = 0;  // 0 = awaiting gear 2, 1 = awaiting gear 10

// Entering calibration must be deliberate: require Btn3 held for N consecutive
// CH3_LONG events before it triggers. CH3_LONG repeats every LONGPRESS_INTERVAL_MS
// (200 ms in multi_btn.c), so 10 ticks ~= 2 s. This is decoupled from the shared
// long-press interval so Btn1/Btn2 jog/shift repeat stays fast.
#define CALIB_ENTER_LONG_TICKS 10
static uint8_t calib_long_count = 0;

static char command_message[CMD_LENGTH] = {};

bool data_process_command = false;

epx_configuration_t epx_configuration;
epx_position_configuration_t epx_position;

bool update_config_flash = false; //Update the flash memory from the main loop
bool update_pos_flash = false; //Update the flash memory from the main loop

bool shift_mode = true; //if true then we are in a gear mode, if false we're in an angle mode
uint8_t long_mode_count = 0;

// ---------------------------------------------------------------------------
// Output + argument helpers
//
// dh_reply() is the single place a command response is formatted and sent. It
// replaces the old sprintf(buff1, ...) + nus_data_send(...) pattern (and the two
// shared global buffers), formatting onto a private stack buffer. Output still
// goes to both the UART console and BLE NUS via nus_data_send(). Responses carry
// no trailing newline; console_print() appends CRLF.
//
// dh_arg_float()/dh_arg_int() parse a numeric argument from the tail of the
// command line (the pointer passed to a command handler already points past the
// command character).
// ---------------------------------------------------------------------------
static void dh_reply(const char *fmt, ...)
{
    char line[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    uint16_t len = (n < (int)sizeof(line)) ? (uint16_t)n : (uint16_t)(sizeof(line) - 1);
    nus_data_send((uint8_t *)line, len);
}

static float dh_arg_float(const char *args)
{
    return strtof(args, NULL);
}

static int32_t dh_arg_int(const char *args)
{
    return (int32_t)atoi(args);
}

void data_handler_button_event_handler(multibtn_event_t evt)
{
    NRF_LOG_INFO("button_event_handler %d", evt);

    // Btn1 = up, Btn2 = down, Btn3 = mode/capture. In calibration the up/down
    // buttons jog the derailleur and Btn3 captures the current gear reference.
    if (calibrating)
    {
        // Btn1 = up, Btn2 = down in terms of *derailleur motion*. The angle
        // increases in the opposite physical direction, so Btn1 (up) jogs the
        // angle negative and Btn2 (down) jogs it positive. Each jog clears any
        // overcurrent latch first so a jog back off a hard stop can't trap us.
        switch (evt)
        {
        case MULTI_BTN_EVENT_CH1_PUSH:  mpos_clear_fault(); mpos_update_angle(false, -(float)CALIB_JOG_FINE);   break;
        case MULTI_BTN_EVENT_CH2_PUSH:  mpos_clear_fault(); mpos_update_angle(false, (float)CALIB_JOG_FINE);    break;
        case MULTI_BTN_EVENT_CH1_LONG:  mpos_clear_fault(); mpos_update_angle(false, -(float)CALIB_JOG_COARSE); break;
        case MULTI_BTN_EVENT_CH2_LONG:  mpos_clear_fault(); mpos_update_angle(false, (float)CALIB_JOG_COARSE);  break;
        case MULTI_BTN_EVENT_CH3_PUSH:  data_handler_calibration_capture();                 break;
        default: break;
        }
        return;
    }

	switch (evt)
	{
	case MULTI_BTN_EVENT_NOTHING:
		break;

	case MULTI_BTN_EVENT_CH1_PUSH:   // up
        data_handler_shift_gear_handler(false, 1);
		break;

	case MULTI_BTN_EVENT_CH2_PUSH:   // down
        data_handler_shift_gear_handler(false, -1);
		break;

	case MULTI_BTN_EVENT_CH3_PUSH:   // mode: toggle gear/angle
        data_handler_shift_mode_handler(false, !shift_mode);
		break;

	case MULTI_BTN_EVENT_CH4_PUSH:
		break;

	case MULTI_BTN_EVENT_CH1_LONG:   // up (repeats while held)
        data_handler_shift_gear_handler(false, 1);
		break;

	case MULTI_BTN_EVENT_CH2_LONG:   // down (repeats while held)
        data_handler_shift_gear_handler(false, -1);
		break;

	case MULTI_BTN_EVENT_CH3_LONG:   // hold mode -> enter guided calibration (deliberate ~2 s hold)
        if (++calib_long_count >= CALIB_ENTER_LONG_TICKS)
        {
            calib_long_count = 0;
            data_handler_calibration_enter();
        }
		break;

	case MULTI_BTN_EVENT_CH4_LONG:
		break;

	case MULTI_BTN_EVENT_CH1_RELEASE:
		break;

	case MULTI_BTN_EVENT_CH2_RELEASE:
		break;

	case MULTI_BTN_EVENT_CH3_RELEASE:
        calib_long_count = 0;   // released before the hold threshold; restart the count
		break;

	case MULTI_BTN_EVENT_CH4_RELEASE:
		break;

	default:
		break;
	}

}

void data_handler_command(const char* p_chars, uint32_t length)
{
    if (length >= sizeof(command_message)) length = sizeof(command_message) - 1;
    memset(command_message, 0, sizeof(command_message));
    memcpy(command_message, p_chars, length);
    data_process_command = true;
}

// ---------------------------------------------------------------------------
// Command dispatch table
//
// Each console/BLE command is one row: the (lower-cased) leading character, the
// handler, and a one-line help string. A handler receives `args`, a pointer to
// the rest of the line *after* the command character, so sub-commands (e.g.
// "g l", "x c") and numeric arguments (e.g. "p1.5") are parsed from there.
// Adding a command is a single row plus a small handler; `h` lists the table.
// ---------------------------------------------------------------------------
typedef void (*dh_handler_t)(const char *args);
typedef struct { char key; dh_handler_t fn; const char *help; } dh_cmd_t;

static void cmd_set_kp(const char *args);
static void cmd_set_ki(const char *args);
static void cmd_set_kd(const char *args);
static void cmd_show_gains(const char *args);
static void cmd_gear(const char *args);
static void cmd_shift(const char *args);
static void cmd_mode(const char *args);
static void cmd_target(const char *args);
static void cmd_front(const char *args);
static void cmd_overshift(const char *args);
static void cmd_fault(const char *args);
static void cmd_force_save(const char *args);
static void cmd_calibrate(const char *args);
static void cmd_telemetry(const char *args);
static void cmd_monitor(const char *args);
static void cmd_events(const char *args);
static void cmd_status(const char *args);
static void cmd_help(const char *args);

static const dh_cmd_t DH_COMMANDS[] = {
    {'p', cmd_set_kp,     "p<f>       set Kp"},
    {'i', cmd_set_ki,     "i<f>       set Ki"},
    {'d', cmd_set_kd,     "d<f>       set Kd"},
    {'k', cmd_show_gains, "k          list gains"},
    {'g', cmd_gear,       "g<n>|l|h|i set gear pos / cal refs / fit"},
    {'s', cmd_shift,      "s<g>|+|-   shift to gear (gear mode)"},
    {'m', cmd_mode,       "m a|g      angle/gear mode"},
    {'t', cmd_target,     "t<f>       target angle (angle mode)"},
    {'b', cmd_front,      "b0|b1      front position select"},
    {'o', cmd_overshift,  "o ...      overshift/dwell table"},
    {'x', cmd_fault,      "x|xl|xc    fault clear / isense limit / count"},
    {'f', cmd_force_save, "fs         force flash save"},
    {'c', cmd_calibrate,  "c          enter/cancel calibration"},
    {'y', cmd_telemetry,  "y<n>       CSV telemetry divider (0 off)"},
    {'v', cmd_monitor,    "v<n>       verbose monitor divider (0 off)"},
    {'e', cmd_events,     "e<mask>    HIL event categories (0 off, 63 all)"},
    {'?', cmd_status,     "?          one-shot status"},
    {'h', cmd_help,       "h          this help"},
};

#define DH_COMMAND_COUNT (sizeof(DH_COMMANDS) / sizeof(DH_COMMANDS[0]))

// Setting a gain persists it (consistent with the gear/overshift/fault commands,
// which also mark the config dirty). The live PID reads epx_configuration.Kp/Ki/Kd
// through bound pointers, so the change also takes effect immediately.
static void cmd_set_kp(const char *args)    { epx_configuration.Kp = dh_arg_float(args); update_config_flash = true; data_handler_show_gains(); }
static void cmd_set_ki(const char *args)    { epx_configuration.Ki = dh_arg_float(args); update_config_flash = true; data_handler_show_gains(); }
static void cmd_set_kd(const char *args)    { epx_configuration.Kd = dh_arg_float(args); update_config_flash = true; data_handler_show_gains(); }
static void cmd_show_gains(const char *args){ (void)args; data_handler_show_gains(); }
static void cmd_gear(const char *args)      { (void)args; data_handler_command_gear_value(); }
static void cmd_overshift(const char *args) { (void)args; data_handler_overshift_command(); }
static void cmd_fault(const char *args)     { (void)args; data_handler_fault_command(); }
static void cmd_force_save(const char *args){ data_handler_force_save(args[0]); }
static void cmd_status(const char *args)    { (void)args; data_handler_print_status(); }

static void cmd_shift(const char *args)
{
    (void)args;
    if (shift_mode) data_handler_shift_gear_handler(true, 0);
}

static void cmd_mode(const char *args)
{
    (void)args;
    data_handler_shift_mode_handler(true, false); // command form: 2nd char picks the mode
}

static void cmd_target(const char *args)
{
    if (!shift_mode) mpos_update_angle(true, dh_arg_float(args));
}

static void cmd_front(const char *args)
{
    epx_position.current_front = (int8_t)dh_arg_int(args);
    if (epx_position.current_front < 0)              epx_position.current_front = 0;
    if (epx_position.current_front >= NUM_FRONT_POS) epx_position.current_front = NUM_FRONT_POS - 1;
    update_pos_flash = true;
}

static void cmd_calibrate(const char *args)
{
    (void)args;
    if (calibrating) data_handler_calibration_cancel();
    else             data_handler_calibration_enter();
}

static void cmd_telemetry(const char *args)
{
    telemetry_set_rate((uint16_t)dh_arg_int(args));
}

static void cmd_monitor(const char *args)
{
    mpos_set_monitor((uint16_t)dh_arg_int(args));
}

static void cmd_events(const char *args)
{
    if (args[0] == '\0')   // bare 'e': report the current mask + legend
    {
        dh_reply("evt mask 0x%02lX (boot1 cal2 save4 turn8 fault16 shift32)",
                 (unsigned long)evt_log_get_mask());
        return;
    }
    uint32_t mask = (uint32_t)strtoul(args, NULL, 0); // accepts decimal or 0x..
    evt_log_set_mask(mask);
    dh_reply("evt mask 0x%02lX", (unsigned long)mask);
}

static void cmd_help(const char *args)
{
    (void)args;
    for (unsigned i = 0; i < DH_COMMAND_COUNT; i++)
    {
        dh_reply("%s", DH_COMMANDS[i].help);
    }
}

void data_handler_command_processor(void)
{
    command_message[0] = (char)tolower((unsigned char)command_message[0]);
    command_message[1] = (char)tolower((unsigned char)command_message[1]);

    char key = command_message[0];
    if (key == '\0') return;   // empty line, nothing to do

    for (unsigned i = 0; i < DH_COMMAND_COUNT; i++)
    {
        if (DH_COMMANDS[i].key == key)
        {
            DH_COMMANDS[i].fn(&command_message[1]);
            return;
        }
    }
    cmd_help(NULL);   // unknown command: show what's available
}

void data_handler_force_save(char command)
{
    if (command == 0x53 || command == 0x73) //if 's' or 'S'
    {
        update_config_flash = true;
    }
}

void data_handler_shift_gear_handler(bool command, int shift_count)
{
    if(shift_mode) //check if we're in shift mode
    {
        int old_gear = epx_position.current_gear;

        if (command) //if there is a command then process and decode
        {
            switch (command_message[1])
            {
            case 0x2B: //+
                epx_position.current_gear++;
                break;
            case 0x2D: //-
                epx_position.current_gear--;
                break;
            default:
                epx_position.current_gear = dh_arg_int(&command_message[1]);
                break;
            }
        } else //no command then process the shift count
        {
            epx_position.current_gear += shift_count;
        }

        //guards to ensure it stays within number of gears
        if(epx_position.current_gear > epx_configuration.num_gears-1)
        {
            epx_position.current_gear = epx_configuration.num_gears-1;
        }

        if(epx_position.current_gear < 0)
        {
            epx_position.current_gear = 0;
        }

        int new_gear = epx_position.current_gear;
        int32_t final_pos = epx_configuration.gear_pos[new_gear];

        // Look up overshift for this gear / front position / direction. It is
        // stored as per-mille of the shift's gear-to-gear distance; multiplying
        // by the signed span gives the overtravel in the direction of travel
        // automatically. overshift_pm==0 => plain move, no dwell.
        int16_t  signed_overshift = 0;
        uint16_t dwell_ticks      = 0;
        int front = epx_position.current_front;
        if (new_gear < NUM_REAR_GEARS && front >= 0 && front < NUM_FRONT_POS && new_gear != old_gear)
        {
            int dir = (new_gear > old_gear) ? DIR_UP : DIR_DOWN;
            overshift_t os = epx_configuration.rear_overshift[new_gear][front][dir];
            if (os.overshift_pm != 0)
            {
                int32_t span = final_pos - epx_configuration.gear_pos[old_gear]; // signed travel distance
                signed_overshift = (int16_t)(((int32_t)os.overshift_pm * span) / 1000);
                dwell_ticks      = (uint16_t)(((int32_t)os.dwell_ms * 256) / 1000);
            }
        }

        NRF_LOG_INFO("Gear %d Angle %ld overshift %d", new_gear, final_pos, signed_overshift);
        mpos_shift_to(final_pos, signed_overshift, dwell_ticks);

        if (new_gear != old_gear)
        {
            evt_log(EVT_SHIFT, "#shift,gear=%d,pos=%ld,over=%d",
                    new_gear, (long)final_pos, signed_overshift);
        }

        // Persist the gear + target the instant we commit to it, not only when the
        // motor settles. Otherwise a reboot before settle reverts to the previous
        // gear and the position is offset. (Turn-count saves on movement keep
        // current_rotations consistent alongside this.)
        if (new_gear != old_gear) update_pos_flash = true;
    } else //if we're in angle mode
    {
        mpos_update_angle(false, (float)shift_count*5);
    }


}

void data_handler_shift_mode_handler(bool command, bool mode)
{
    if (command) //if we are dealing with a text command we'll do some decode
    {
        switch (command_message[1])
        {
        case 0x61: //a
            shift_mode = false;
            break;

        case 0x67: //g
            shift_mode = true;
            break;

        default:
            break;
        }
    } else // we're looking at a direct mode switch
    {
        shift_mode = mode;
    }

    dh_reply("%s", shift_mode ? "Gear Mode" : "Angle Mode");
}

void data_handler_long_mode_handler(bool long_press)
{
    if (long_press)
    {
        long_mode_count++;
    } else //reset on release
    {
        long_mode_count = 0;
    }

    if(long_mode_count > 4)
    {
        long_mode_count = 0;
        data_handler_shift_mode_handler(false, false); //change modes
    }
}

void data_handler_command_gear_value(void)
{
    char sub = command_message[1];
    update_config_flash = true;   // most subcommands persist; cleared below if not

    if (sub >= '1' && sub <= '9')          // g1..g9  -> gears 1..9
    {
        epx_configuration.gear_pos[sub - '1'] = dh_arg_int(&command_message[2]);
    }
    else if (sub >= 'a' && sub <= 'e')     // ga..ge  -> gears 10..14
    {
        epx_configuration.gear_pos[9 + (sub - 'a')] = dh_arg_int(&command_message[2]);
    }
    else if (sub == 'f')                   // gf<n>   -> number of gears
    {
        epx_configuration.num_gears = dh_arg_int(&command_message[2]);
    }
    else if (sub == 'l')                   // gl: capture current angle as LOW ref (gear 2)
    {
        update_config_flash = false;       // reference is RAM-only until interpolation runs
        ref_lo = (int32_t)mpos_last_angle();
        ref_lo_set = true;
        dh_reply("Low ref (gear %d): %ld", GEAR_REF_LO_IDX + 1, ref_lo);
        return;
    }
    else if (sub == 'h')                   // gh: capture current angle as HIGH ref (gear 10)
    {
        update_config_flash = false;
        ref_hi = (int32_t)mpos_last_angle();
        ref_hi_set = true;
        dh_reply("High ref (gear %d): %ld", GEAR_REF_HI_IDX + 1, ref_hi);
        return;
    }
    else if (sub == 'i')                   // gi: interpolate all gears from the two refs
    {
        data_handler_compute_gears();
        return;
    }
    else                                   // unknown subcommand: just echo the table
    {
        update_config_flash = false;
    }

    data_handler_command_gear_value_print();
}

void data_handler_command_gear_value_print(void)
{
    dh_reply("Gear 1: %ld, %ld, %ld, %ld, %ld, %ld",
             epx_configuration.gear_pos[0], epx_configuration.gear_pos[1], epx_configuration.gear_pos[2],
             epx_configuration.gear_pos[3], epx_configuration.gear_pos[4], epx_configuration.gear_pos[5]);
    dh_reply("Gear 7: %ld, %ld, %ld, %ld, %ld, %ld",
             epx_configuration.gear_pos[6], epx_configuration.gear_pos[7], epx_configuration.gear_pos[8],
             epx_configuration.gear_pos[9], epx_configuration.gear_pos[10], epx_configuration.gear_pos[11]);
}

void data_handler_compute_gears(void)
{
    if (!ref_lo_set || !ref_hi_set)
    {
        evt_log(EVT_CAL, "#cal,step=fit,ok=0");
        dh_reply("Capture gl and gh first");
        return;
    }

    // Affine-fit the (non-linear) nominal cog profile through the two captures.
    bool ok = gears_fit_profile(epx_configuration.gear_pos,
                                epx_configuration.num_gears,
                                gear_profile_nominal,
                                GEAR_REF_LO_IDX, ref_lo,
                                GEAR_REF_HI_IDX, ref_hi);
    if (!ok)
    {
        evt_log(EVT_CAL, "#cal,step=fit,ok=0");
        dh_reply("Fit failed (num_gears %ld?)", epx_configuration.num_gears);
        return;
    }
    evt_log(EVT_CAL, "#cal,step=fit,ok=1");

    // Persist the captured references so a re-fit needs no re-jog.
    epx_configuration.ref_lo     = ref_lo;
    epx_configuration.ref_hi     = ref_hi;
    epx_configuration.ref_lo_idx = GEAR_REF_LO_IDX;
    epx_configuration.ref_hi_idx = GEAR_REF_HI_IDX;

    update_config_flash = true; // persist the computed positions
    data_handler_command_gear_value_print();
}

void data_handler_overshift_command(void)
{
    int g = 0, f = 0, d = 0, over = 0, dwell = 0;
    int got = sscanf(command_message + 1, "%d %d %d %d %d", &g, &f, &d, &over, &dwell);

    if (got < 5) // not a full set => list the table for the current front position
    {
        int front = epx_position.current_front;
        dh_reply("Overshift front %d (gear: up/dn pm,dwell):", front);
        for (int i = 0; i < NUM_REAR_GEARS; i++)
        {
            overshift_t u  = epx_configuration.rear_overshift[i][front][DIR_UP];
            overshift_t dn = epx_configuration.rear_overshift[i][front][DIR_DOWN];
            dh_reply("g%d: u %d,%d  d %d,%d", i + 1,
                     u.overshift_pm, u.dwell_ms, dn.overshift_pm, dn.dwell_ms);
        }
        return;
    }

    int gi = g - 1; // command is 1-based gear
    if (gi < 0 || gi >= NUM_REAR_GEARS || f < 0 || f >= NUM_FRONT_POS || d < 0 || d >= NUM_DIRS)
    {
        dh_reply("Bad o args: gear 1-%d front 0-%d dir 0/1", NUM_REAR_GEARS, NUM_FRONT_POS - 1);
        return;
    }

    epx_configuration.rear_overshift[gi][f][d].overshift_pm = (int16_t)over;
    epx_configuration.rear_overshift[gi][f][d].dwell_ms     = (int16_t)dwell;
    update_config_flash = true;

    dh_reply("Set g%d f%d %s pm %d dwell %d", g, f, d ? "dn" : "up", over, dwell);
}

void data_handler_fault_command(void)
{
    switch (command_message[1])
    {
    case 0x6C: //l set ISENSE limit (raw counts)
        epx_configuration.isense_limit = (int16_t)dh_arg_int(&command_message[2]);
        update_config_flash = true;
        dh_reply("ISENSE limit %d", epx_configuration.isense_limit);
        break;
    case 0x63: //c set fault sample count
        epx_configuration.isense_fault_count = (uint16_t)dh_arg_int(&command_message[2]);
        update_config_flash = true;
        dh_reply("ISENSE count %u", epx_configuration.isense_fault_count);
        break;
    default: //clear the latched fault
        mpos_clear_fault();
        dh_reply("Fault cleared");
        break;
    }
}

void data_handler_show_gains(void)
{
    dh_reply("Gains: Kp: %.3f, Ki: %.3f, Kd: %.3f",
             epx_configuration.Kp, epx_configuration.Ki, epx_configuration.Kd);
}

void data_handler_print_status(void)
{
    const char *mode = calibrating ? "CAL" : (shift_mode ? "GEAR" : "ANGLE");
    int32_t pos = (int32_t)mpos_last_angle();
    int32_t tgt = mpos_subtarget();
    dh_reply("%s g%d pos %ld tgt %ld err %ld I %d %s%s",
             mode, epx_position.current_gear, pos, tgt, tgt - pos,
             mpos_isense(), mpos_state() ? "MOV" : "HLD",
             mpos_is_faulted() ? " FAULT" : "");
}

void data_handler_calibration_enter(void)
{
    calibrating = true;
    calib_step  = 0;
    calib_long_count = 0;  // consume the hold so it doesn't immediately re-trigger
    shift_mode  = false;   // angle mode so the buttons jog and gears don't shift

    // Escape hatch: a bad stored position can drive into an end-of-travel hard
    // stop on boot and latch an overcurrent fault, which would otherwise inhibit
    // the jog we need to recover. Clear the fault and hold the *current* position
    // so we stop pushing into the stop; the user then jogs away and captures.
    mpos_clear_fault();
    mpos_update_angle(true, mpos_last_angle());

    evt_log(EVT_CAL, "#cal,step=enter");
    dh_reply("CAL 1/2: jog to GEAR %d (b1 up / b2 down), b3 capture; 'c' cancels",
             GEAR_REF_LO_IDX + 1);
}

void data_handler_calibration_cancel(void)
{
    calibrating = false;
    evt_log(EVT_CAL, "#cal,step=cancel");
    dh_reply("CAL cancelled");
}

void data_handler_calibration_capture(void)
{
    if (calib_step == 0)
    {
        ref_lo     = (int32_t)mpos_last_angle();
        ref_lo_set = true;
        calib_step = 1;
        evt_log(EVT_CAL, "#cal,step=lo,angle=%ld", (long)ref_lo);
        dh_reply("Gear %d = %ld. CAL 2/2: jog to GEAR %d, b3 capture",
                 GEAR_REF_LO_IDX + 1, ref_lo, GEAR_REF_HI_IDX + 1);
    }
    else
    {
        ref_hi      = (int32_t)mpos_last_angle();
        ref_hi_set  = true;
        calibrating = false;
        evt_log(EVT_CAL, "#cal,step=hi,angle=%ld", (long)ref_hi);
        dh_reply("Gear %d = %ld. Computing gears...", GEAR_REF_HI_IDX + 1, ref_hi);
        data_handler_compute_gears();   // fits profile through the two refs, persists, echoes
    }
}

void data_handler_sch_execute(void)
{
    if (data_process_command)
    {
        data_handler_command_processor();
        data_process_command = false;
    }

    if (mpos_monitor_due())   // low-rate human-readable status (v<n>)
    {
        data_handler_print_status();
    }

    if(update_config_flash)
    {
        NRF_LOG_INFO("data_handler_sch_execute config flash update");
        update_config_flash = false;
        mem_epx_config_update(epx_configuration);
        evt_log(EVT_SAVE, "#save,what=cfg,rot=%ld,gear=%d",
                (long)epx_position.current_rotations, (int)epx_position.current_gear);
    }

    if(update_pos_flash)
    {
        NRF_LOG_INFO("data_handler_sch_execute position flash update");
        update_pos_flash = false;
        mem_epx_position_update(epx_position);
        evt_log(EVT_SAVE, "#save,what=pos,rot=%ld,gear=%d",
                (long)epx_position.current_rotations, (int)epx_position.current_gear);
    }

}

void data_handler_get_flash_values(void)
{
    epx_configuration = tm_fds_epx_config(); //get configuration from titanmem
    epx_position = tm_fds_epx_position(); //get position data from titanmem
    //link the live gains to the PID (owned by mpos) so it reads the stored config directly
    mpos_link_gains(&epx_configuration.Kp, &epx_configuration.Ki, &epx_configuration.Kd);
    mpos_link_overcurrent(&epx_configuration.isense_limit, &epx_configuration.isense_fault_count);
    mpos_link_memory(&epx_position);//link the local value to the mpos controller
}

void data_handler_set_boot_target(void)
{
    // Source of truth on boot:
    //  - current_rotations (from the saved position record) fixes ABSOLUTE position;
    //  - current_gear (also from the position record) + the calibrated gear table
    //    give the TARGET.
    // The saved target_angle is ignored when the gears are filled in. Call after
    // mpos_init() so this overrides mpos's default target seed.
    if (epx_configuration.num_gears > 0)
    {
        int g = epx_position.current_gear;
        if (g < 0) g = 0;
        if (g > epx_configuration.num_gears - 1) g = epx_configuration.num_gears - 1;
        mpos_update_angle(true, (float)epx_configuration.gear_pos[g]);
    }
    // else: uncalibrated -> leave mpos holding its loaded target_angle (no gear table yet).
}

void data_handler_req_update_position_flash(void)
{
    update_pos_flash = true;
}
