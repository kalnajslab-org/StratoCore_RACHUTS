/*
 *  ManualMotion.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoRachuts.h"

enum ManualMotionStates_t {
    ST_ENTRY,
    ST_SEND_RA,
    ST_WAIT_RAACK,
    ST_START_MOTION,
    ST_VERIFY_MOTION,
    ST_MONITOR_MOTION,
    ST_TM_ACK,
};

static ManualMotionStates_t manualmotion_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoRachuts::Flight_ManualMotion(bool restart_state)
{
    if (restart_state) manualmotion_state = ST_ENTRY;

    // Cancel Motion (TC 11, which has already sent MCB_CANCEL_MOTION) is checked
    // here in every state, not just ST_MONITOR_MOTION, so a cancel sent while
    // waiting for the RA ack isn't dropped and the motion then started anyway.
    // Either way the schedule is cleared so this motion's pending resend timers
    // can't fire into the next one.
    if (CheckAction(ACTION_MOTION_STOP)) {
        switch (manualmotion_state) {
        case ST_ENTRY:
        case ST_SEND_RA:
        case ST_WAIT_RAACK:
            // motion not yet commanded
            scheduler.ClearSchedule();
            resend_attempted = false;
            SendTextTM("Manual motion cancelled before start", WARN);
            return true;
        case ST_TM_ACK:
            // motion already finished, nothing to stop
            SendTextTM("Cancel motion: no motion to cancel, manual motion already complete", WARN);
            break;
        default:
            // todo: verification of motion stop
            scheduler.ClearSchedule();
            resend_attempted = false;
            SendTextTM("Commanded motion stop", FINE);
            return true;
        }
    }

    switch (manualmotion_state) {
    case ST_ENTRY:
    case ST_SEND_RA:
        RA_ack_flag = NO_ACK;
        ZephyrTXpoke(ZEPHYRTX_RA);
        manualmotion_state = ST_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, ZEPHYR_RESEND_TIMEOUT);
        log_nominal("Sending RA");
        break;

    case ST_WAIT_RAACK:
        if (ra_ack_override) // TC-commanded bypass of the RA ack requirement (emergency use)
            RA_ack_flag = ACK;
        if (ACK == RA_ack_flag) {
            manualmotion_state = ST_START_MOTION;
            resend_attempted = false;
            log_nominal("RA ACK");
        } else if (NAK == RA_ack_flag) {
            resend_attempted = false;
            SendTextTM("Cannot perform motion, RA NAK", WARN);
            return true;
        } else if (CheckAction(RESEND_RA)) {
            if (!resend_attempted) {
                resend_attempted = true;
                manualmotion_state = ST_SEND_RA;
            } else {
                SendTextTM("Never received RAAck", WARN);
                resend_attempted = false;
                return true;
            }
        }
        break;

    case ST_START_MOTION:
        if (mcb_motion_ongoing) {
            SendTextTM("Motion commanded while motion ongoing", WARN);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            break;
        }

        if (StartMCBMotion()) {
            manualmotion_state = ST_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
        } else {
            SendTextTM("Motion start error", WARN);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
        }
        break;

    case ST_VERIFY_MOTION:
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            motion_deadline_ms = millis() + max_profile_seconds * 1000UL;
            manualmotion_state = ST_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                manualmotion_state = ST_START_MOTION;
            } else {
                resend_attempted = false;
                SendTextTM("MCB never confirmed motion", WARN);
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            }
        }
        break;

    case ST_MONITOR_MOTION:
        if ((int32_t) (millis() - motion_deadline_ms) >= 0) {
            SendMCBTM("MCBREPORT", CRIT, "MCB Motion took longer than expected");
            mcbComm.TX_ASCII(MCB_CANCEL_MOTION);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            break;
        }

        if (!mcb_motion_ongoing) {
            SendMCBTM("MCBREPORT", FINE, "Finished commanded manual motion");
            manualmotion_state = ST_TM_ACK;
            scheduler.AddAction(RESEND_TM, ZEPHYR_RESEND_TIMEOUT);
        }
        break;

    case ST_TM_ACK:
        if (ACK == TM_ack_flag) {
            log_nominal("Zephyr ACKed motion TM");
            return true;
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // attempt one resend
            log_error("Needed to resend TM");
            ZephyrTXpoke(ZEPHYRTX_TM); // message is still saved in XMLWriter, no need to reconstruct
            return true;
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}