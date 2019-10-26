/*
 *  Flight_ReDock.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoPIB.h"

enum InternalStates_t {
    ST_ENTRY,
    ST_IDLE,
    ST_START_MOTION,
    ST_VERIFY_MOTION,
    ST_MONITOR_MOTION,
    ST_CHECK_PU,
    ST_WAIT_PU,
};

static InternalStates_t internal_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoPIB::Flight_ReDock(bool restart_state)
{
    if (restart_state) internal_state = ST_ENTRY;

    switch (internal_state) {
    case ST_ENTRY:
        internal_state = ST_IDLE;
        SetAction(ACTION_REEL_OUT);
        scheduler.AddAction(ACTION_IN_NO_LW, 30);
        scheduler.AddAction(ACTION_CHECK_PU, 60);
        break;

    case ST_IDLE:
        if (CheckAction(ACTION_REEL_OUT)) {
            internal_state = ST_START_MOTION;
            mcb_motion = MOTION_REEL_OUT;
            resend_attempted = false;
        } else if (CheckAction(ACTION_IN_NO_LW)) {
            internal_state = ST_START_MOTION;
            mcb_motion = MOTION_IN_NO_LW;
            resend_attempted = false;
        } else if (CheckAction(ACTION_CHECK_PU)) {
            internal_state = ST_CHECK_PU;
            resend_attempted = false;
        }

    case ST_START_MOTION:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion commanded while motion ongoing");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
        }

        if (StartMCBMotion()) {
            inst_substate = ST_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
        } else {
            ZephyrLogWarn("Motion start error");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
        }
        break;

    case ST_VERIFY_MOTION:
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            inst_substate = ST_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = ST_START_MOTION;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            }
        }
        break;

    case ST_MONITOR_MOTION:
        if (CheckAction(ACTION_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            return true;
            break;
        }

        if (!mcb_motion_ongoing) {
            scheduler.AddAction(RESEND_TM, ZEPHYR_RESEND_TIMEOUT);
        }
        break;

    case ST_CHECK_PU:
        puComm.TX_ASCII(PU_SEND_STATUS);
        scheduler.AddAction(RESEND_PU_CHECK, PU_RESEND_TIMEOUT);
        inst_substate = ST_WAIT_PU;
        break;

    case ST_WAIT_PU:
        if (pib_config.pu_docked) {
            snprintf(log_array, LOG_ARRAY_SIZE, "PU status: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", pu_status.time, pu_status.v_battery, pu_status.i_charge, pu_status.therm1, pu_status.therm2, pu_status.heater_stat);
            ZephyrLogFine(log_array);
            mcbComm.TX_ASCII(MCB_ZERO_REEL);
            return true;
            break;
        }

        if (CheckAction(RESEND_PU_CHECK)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = ST_CHECK_PU;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not responding to status request");
                return true;
            }
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}