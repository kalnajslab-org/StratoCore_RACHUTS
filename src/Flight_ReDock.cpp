/*
 *  Flight_ReDock.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoRachuts.h"

// Earliest times, measured from the start of the redock, for the reel-in (no
// level wind) and the PU check. Each step also waits for the previous motion to
// finish, so a slow reel-out delays the reel-in rather than skipping it.
#define REDOCK_REEL_IN_DELAY_MS     30000UL
#define REDOCK_CHECK_PU_DELAY_MS    60000UL

enum ReDockStates_t {
    ST_ENTRY,
    ST_WAIT_STEP,
    ST_START_MOTION,
    ST_VERIFY_MOTION,
    ST_MONITOR_MOTION,
    ST_CHECK_PU,
    ST_WAIT_PU,
};

static ReDockStates_t redock_state = ST_ENTRY;
static bool resend_attempted = false;
static bool motion_sent = false; // true once this redock has commanded a reel motion
static uint32_t redock_start_ms = 0;

bool StratoRachuts::Flight_ReDock(bool restart_state)
{
    if (restart_state) {
        redock_state = ST_ENTRY;
        motion_sent = false;
    }

    // Cancel Motion (TC 11, which has already sent MCB_CANCEL_MOTION) is checked
    // here in every state, not just ST_MONITOR_MOTION, so a standalone redock
    // (TC 142) waiting between its steps can still be cancelled.
    // Inside a profile, Flight_Profile consumes the cancel before calling this.
    if (CheckAction(ACTION_MOTION_STOP)) {
        if (ST_CHECK_PU == redock_state || ST_WAIT_PU == redock_state) {
            // both motions finished, only the PU check is left
            SendTextTM("Cancel motion: no motion to cancel, redock motion already complete", WARN);
        } else if (!motion_sent) {
            // reel-out not yet sent, so the PU hasn't moved: drop any pending
            // resend timers and return to FLM_IDLE
            scheduler.ClearSchedule();
            resend_attempted = false;
            SendTextTM("Redock cancelled before reel out", WARN);
            return true;
        } else {
            SendTextTM("Commanded motion stop in redock", WARN);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            return false;
        }
    }

    switch (redock_state) {
    case ST_ENTRY:
        redock_start_ms = millis();
        mcb_motion = MOTION_REEL_OUT;
        resend_attempted = false;
        redock_state = ST_START_MOTION;
        break;

    case ST_WAIT_STEP:
        // the motion that just finished decides the next step
        if (MOTION_REEL_OUT == mcb_motion) {
            if ((int32_t) (millis() - (redock_start_ms + REDOCK_REEL_IN_DELAY_MS)) >= 0) {
                mcb_motion = MOTION_IN_NO_LW;
                resend_attempted = false;
                redock_state = ST_START_MOTION;
            }
        } else if (MOTION_IN_NO_LW == mcb_motion) {
            if ((int32_t) (millis() - (redock_start_ms + REDOCK_CHECK_PU_DELAY_MS)) >= 0) {
                resend_attempted = false;
                redock_state = ST_CHECK_PU;
            }
        } else {
            SendTextTM("Unknown motion finished in redock", CRIT);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
        }
        break;

    case ST_START_MOTION:
        if (mcb_motion_ongoing) {
            SendTextTM("Motion commanded while motion ongoing", WARN);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            break;
        }

        motion_sent = true;
        if (StartMCBMotion()) {
            redock_state = ST_VERIFY_MOTION;
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
            redock_state = ST_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                redock_state = ST_START_MOTION;
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
            redock_state = ST_WAIT_STEP;
        }
        break;

    case ST_CHECK_PU:
        puComm.TX_ASCII(RPU_SEND_STATUS);
        scheduler.AddAction(RESEND_PU_CHECK, RPU_RECEIVE_TIMEOUT);
        redock_state = ST_WAIT_PU;
        break;

    case ST_WAIT_PU:
        if (pibConfigs.pu_docked.Read()) {
            force_rachutsreport = true; // mode loop sends the status report
            mcbComm.TX_ASCII(MCB_ZERO_REEL);
            return true;
            break;
        }

        if (CheckAction(RESEND_PU_CHECK)) {
            if (!resend_attempted) {
                resend_attempted = true;
                redock_state = ST_CHECK_PU;
            } else {
                resend_attempted = false;
                SendTextTM("PU not responding to status request", WARN);
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