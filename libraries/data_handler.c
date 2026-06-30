/**
 * Copyright (c) 2019 - 2022, TITAN LAB INC
 *
 * All rights reserved.
 * 
 *
 */
#include <stdio.h>
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

//nus buffers
char buff1[80];
char buff2[80];

bool update_config_flash = false; //Update the falsh memory from the main loop
bool update_pos_flash = false; //Update the falsh memory from the main loop

bool shift_mode = true; //if true then we are in a gear mode, if false we're in an angle mode 
uint8_t long_mode_count = 0;

uint32_t dh_debug_counter = 0;

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

void data_handler_command_processor(void)
{
    command_message[0] = tolower(command_message[0]);
    command_message[1] = tolower(command_message[1]);
    // NRF_LOG_INFO("command processor: %s", command_message);
    switch (command_message[0])
    {
    
    case 0x66: //f force save coefficients
        NRF_LOG_INFO("little f");
        data_handler_force_save(command_message[1]);
        break;

    case 0x67: //g Set the position for each Gear
        NRF_LOG_INFO("little g");
        data_handler_command_gear_value();
        break;
    
    case 0x6B: //k List Gains
        NRF_LOG_INFO("little k");
        data_handler_show_gains();
        break;

    case 0x6D: //g Set the position for each Gear
        NRF_LOG_INFO("little m");
        data_handler_shift_mode_handler(true, false); // if the command (first) is true the mode (2nd) is ignored
        break;

    case 0x70: //p Set Kp
        NRF_LOG_INFO("little P");
        epx_configuration.Kp =  data_handler_command_float_return(1);
        data_handler_show_gains();
        break;

    case 0x69: //i set Ki
        NRF_LOG_INFO("little i");
        epx_configuration.Ki =  data_handler_command_float_return(1);
        data_handler_show_gains();
        break;

    case 0x64: //d Set Kd
        NRF_LOG_INFO("little d");
        epx_configuration.Kd =  data_handler_command_float_return(1);
        data_handler_show_gains();
        break;

    case 0x73: //s 
        NRF_LOG_INFO("little s");
        if (shift_mode)
        {
            data_handler_shift_gear_handler(true, 0);
        }
        break;

    case 0x74: //t Target Angle in degrees
        NRF_LOG_INFO("little t");
        if (!shift_mode)
        {
            mpos_update_angle(true, data_handler_command_float_return(1));
        }
        break;

    case 0x79: //y Telemetry stream: y<divider> (y0 off, y1 full rate, y4 ~64Hz)
        NRF_LOG_INFO("little y");
        telemetry_set_rate((uint16_t)data_handler_command_number_return(1));
        break;

    case 0x6F: //o Overshift/dwell: "o <gear> <front> <dir> <permille> <dwell_ms>"
        NRF_LOG_INFO("little o");
        data_handler_overshift_command();
        break;

    case 0x78: //x Fault: 'x' clear, 'xl <n>' set ISENSE limit, 'xc <n>' set count
        NRF_LOG_INFO("little x");
        data_handler_fault_command();
        break;

    case 0x62: //b Front-derailleur position select: b0 / b1 (provision)
        NRF_LOG_INFO("little b");
        epx_position.current_front = (int8_t)data_handler_command_number_return(1);
        if (epx_position.current_front < 0) epx_position.current_front = 0;
        if (epx_position.current_front >= NUM_FRONT_POS) epx_position.current_front = NUM_FRONT_POS - 1;
        update_pos_flash = true;
        break;

    case 0x63: //c Calibration mode: enter, or cancel if already calibrating
        NRF_LOG_INFO("little c");
        if (calibrating) data_handler_calibration_cancel();
        else             data_handler_calibration_enter();
        break;

    case 0x76: //v Verbose monitor: v<divider> human-readable status (v0 off, v128 ~2Hz)
        NRF_LOG_INFO("little v");
        mpos_set_monitor((uint16_t)data_handler_command_number_return(1));
        break;

    case 0x3F: //? one-shot status line
        data_handler_print_status();
        break;

    default:
        break;
    }

}

float data_handler_command_float_return(uint8_t offset)
{
    char temp_array[CMD_LENGTH];
    strncpy(temp_array, command_message+offset, sizeof(temp_array)-1);
    temp_array[sizeof(temp_array)-1] = '\0';
    float calibration_coefficient = strtof(temp_array, NULL);
    return(calibration_coefficient);
}

int32_t data_handler_command_number_return(uint8_t offset)
{
    char temp_array[CMD_LENGTH];
    int32_t x;
    strncpy(temp_array, command_message+offset, sizeof(temp_array)-1);
    temp_array[sizeof(temp_array)-1] = '\0';
    x = atoi(temp_array);
    NRF_LOG_INFO("value, %ld", x);
    return(x);
}

void data_handler_force_save(char command)
{
    if (command == 0x53 || command == 0x73) //if 's' or 'S'
    {
        update_config_flash = true;
    }
    NRF_LOG_INFO("Force save");
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
                epx_position.current_gear = data_handler_command_number_return(1);
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
            sprintf(buff1, "Angle Mode");
            break;

        case 0x67: //g
            shift_mode = true;
            sprintf(buff1, "Gear Mode");
            break; 

        default:
            break;
        }
    } else // we're looking at a direct mode switch
    {
        shift_mode = mode;
    }

    NRF_LOG_INFO(" %s %d" , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
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
    update_config_flash = true;

    switch (command_message[1])
    {
    case 0x31: //1
        epx_configuration.gear_pos[0] = data_handler_command_number_return(2);
        break;
    case 0x32: //2
        epx_configuration.gear_pos[1] = data_handler_command_number_return(2);
        break;
    case 0x33: //3
        epx_configuration.gear_pos[2] = data_handler_command_number_return(2);
        break;
    case 0x34: //4
        epx_configuration.gear_pos[3] = data_handler_command_number_return(2);
        break;
    case 0x35: //5
        epx_configuration.gear_pos[4] = data_handler_command_number_return(2);
        break;
    case 0x36: //6
        epx_configuration.gear_pos[5] = data_handler_command_number_return(2);
        break;
    case 0x37: //7
        epx_configuration.gear_pos[6] = data_handler_command_number_return(2);
        break;
    case 0x38: //8
        epx_configuration.gear_pos[7] = data_handler_command_number_return(2);
        break;
    case 0x39: //9
        epx_configuration.gear_pos[8] = data_handler_command_number_return(2);
        break;
    case 0x61: //10 - a
        epx_configuration.gear_pos[9] = data_handler_command_number_return(2);
        break;
    case 0x62: //11 - b
        epx_configuration.gear_pos[10] = data_handler_command_number_return(2);
        break;
    case 0x63: //12 - c
        epx_configuration.gear_pos[11] = data_handler_command_number_return(2);
        break;
    case 0x64: //13 - d
        epx_configuration.gear_pos[12] = data_handler_command_number_return(2);
        break;
    case 0x65: //14 - e
        epx_configuration.gear_pos[13] = data_handler_command_number_return(2);
        break;
    case 0x66: //f
        epx_configuration.num_gears = data_handler_command_number_return(2);
        NRF_LOG_INFO(" %d" , epx_configuration.num_gears);
        break;
    case 0x6C: //l capture current angle as the LOW reference (gear 2)
        update_config_flash = false; //reference is RAM-only until interpolation runs
        ref_lo = (int32_t)mpos_last_angle();
        ref_lo_set = true;
        sprintf(buff1, "Low ref (gear %d): %ld", GEAR_REF_LO_IDX + 1, ref_lo);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    case 0x68: //h capture current angle as the HIGH reference (gear 10)
        update_config_flash = false;
        ref_hi = (int32_t)mpos_last_angle();
        ref_hi_set = true;
        sprintf(buff1, "High ref (gear %d): %ld", GEAR_REF_HI_IDX + 1, ref_hi);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    case 0x69: //i interpolate all gears from the two captured references
        data_handler_compute_gears();
        return;
    default:
        update_config_flash = false; //if it wasn't the other cases, don't update the flash memory
        break;
    }
    data_handler_command_gear_value_print();
}

void data_handler_command_gear_value_print(void)
{
    sprintf(buff1, "Gear 1: %ld, %ld, %ld, %ld, %ld, %ld",epx_configuration.gear_pos[0], epx_configuration.gear_pos[1], epx_configuration.gear_pos[2], epx_configuration.gear_pos[3], epx_configuration.gear_pos[4], epx_configuration.gear_pos[5]);
    sprintf(buff2, "Gear 7: %ld, %ld, %ld, %ld, %ld, %ld",epx_configuration.gear_pos[6], epx_configuration.gear_pos[7], epx_configuration.gear_pos[8], epx_configuration.gear_pos[9], epx_configuration.gear_pos[10], epx_configuration.gear_pos[11]);

    NRF_LOG_INFO(" %s %d" , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
    nus_data_send((uint8_t *)buff2, strlen(buff2));
}

void data_handler_compute_gears(void)
{
    if (!ref_lo_set || !ref_hi_set)
    {
        sprintf(buff1, "Capture gl and gh first");
        nus_data_send((uint8_t *)buff1, strlen(buff1));
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
        sprintf(buff1, "Fit failed (num_gears %ld?)", epx_configuration.num_gears);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    }

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
        sprintf(buff1, "Overshift front %d (gear: up/dn pm,dwell):", front);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        for (int i = 0; i < NUM_REAR_GEARS; i++)
        {
            overshift_t u = epx_configuration.rear_overshift[i][front][DIR_UP];
            overshift_t dn = epx_configuration.rear_overshift[i][front][DIR_DOWN];
            sprintf(buff1, "g%d: u %d,%d  d %d,%d", i + 1,
                    u.overshift_pm, u.dwell_ms, dn.overshift_pm, dn.dwell_ms);
            nus_data_send((uint8_t *)buff1, strlen(buff1));
        }
        return;
    }

    int gi = g - 1; // command is 1-based gear
    if (gi < 0 || gi >= NUM_REAR_GEARS || f < 0 || f >= NUM_FRONT_POS || d < 0 || d >= NUM_DIRS)
    {
        sprintf(buff1, "Bad o args: gear 1-%d front 0-%d dir 0/1", NUM_REAR_GEARS, NUM_FRONT_POS - 1);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    }

    epx_configuration.rear_overshift[gi][f][d].overshift_pm = (int16_t)over;
    epx_configuration.rear_overshift[gi][f][d].dwell_ms     = (int16_t)dwell;
    update_config_flash = true;

    sprintf(buff1, "Set g%d f%d %s pm %d dwell %d", g, f, d ? "dn" : "up", over, dwell);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_fault_command(void)
{
    switch (command_message[1])
    {
    case 0x6C: //l set ISENSE limit (raw counts)
        epx_configuration.isense_limit = (int16_t)data_handler_command_number_return(2);
        update_config_flash = true;
        sprintf(buff1, "ISENSE limit %d", epx_configuration.isense_limit);
        break;
    case 0x63: //c set fault sample count
        epx_configuration.isense_fault_count = (uint16_t)data_handler_command_number_return(2);
        update_config_flash = true;
        sprintf(buff1, "ISENSE count %u", epx_configuration.isense_fault_count);
        break;
    default: //clear the latched fault
        mpos_clear_fault();
        sprintf(buff1, "Fault cleared");
        break;
    }
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_show_gains(void)
{
    sprintf(buff1, "Gains: Kp: %.3f, Ki: %.3f, Kd: %.3f",epx_configuration.Kp, epx_configuration.Ki, epx_configuration.Kd);
    NRF_LOG_INFO(" %s " , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_print_status(void)
{
    const char *mode = calibrating ? "CAL" : (shift_mode ? "GEAR" : "ANGLE");
    int32_t pos = (int32_t)mpos_last_angle();
    int32_t tgt = mpos_subtarget();
    sprintf(buff1, "%s g%d pos %ld tgt %ld err %ld I %d %s%s",
            mode, epx_position.current_gear, pos, tgt, tgt - pos,
            mpos_isense(), mpos_state() ? "MOV" : "HLD",
            mpos_is_faulted() ? " FAULT" : "");
    nus_data_send((uint8_t *)buff1, strlen(buff1));
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

    sprintf(buff1, "CAL 1/2: jog to GEAR %d (b1 up / b2 down), b3 capture; 'c' cancels",
            GEAR_REF_LO_IDX + 1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_calibration_cancel(void)
{
    calibrating = false;
    sprintf(buff1, "CAL cancelled");
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_calibration_capture(void)
{
    if (calib_step == 0)
    {
        ref_lo     = (int32_t)mpos_last_angle();
        ref_lo_set = true;
        calib_step = 1;
        sprintf(buff1, "Gear %d = %ld. CAL 2/2: jog to GEAR %d, b3 capture",
                GEAR_REF_LO_IDX + 1, ref_lo, GEAR_REF_HI_IDX + 1);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
    }
    else
    {
        ref_hi      = (int32_t)mpos_last_angle();
        ref_hi_set  = true;
        calibrating = false;
        sprintf(buff1, "Gear %d = %ld. Computing gears...", GEAR_REF_HI_IDX + 1, ref_hi);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        data_handler_compute_gears();   // fits profile through the two refs, persists, echoes
    }
}

void data_handler_sch_execute(void)
{

    if (data_process_command)
    {
        data_handler_command_processor();
        data_process_command = false;
        NRF_LOG_INFO("data_process_command, %d", data_process_command);
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
    }

    if(update_pos_flash)
    {
        NRF_LOG_INFO("data_handler_sch_execute position flash update");
        update_pos_flash = false;
        mem_epx_position_update(epx_position);
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



