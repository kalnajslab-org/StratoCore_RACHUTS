/*
 *  Flight.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file implements the RACHuTS flight mode.
 */

#include "StratoPIB.h"

// Flight mode states, FLA = autonomous, FLM = manual, FL = general
enum FLStates_t : uint8_t {
    FL_ENTRY = MODE_ENTRY,

    // before anything else
    FL_GPS_WAIT,

    // manual
    FLM_IDLE,
    FLM_SEND_RA,
    FLM_WAIT_RAACK,
    FLM_START_MOTION,
    FLM_VERIFY_MOTION,
    FLM_MONITOR_MOTION,

    // autonomous
    FLA_IDLE,

    // general off-nominal states
    FL_ERROR_LOOP,
    FL_SHUTDOWN_LOOP,

    FL_ERROR_LANDING = MODE_ERROR,
    FL_SHUTDOWN = MODE_SHUTDOWN,
    FL_EXIT = MODE_EXIT
};

// this function is called at the defined rate
//  * when flight mode is entered, it will start in FL_ENTRY state
//  * it is then up to this function to change state as needed by updating the inst_substate variable
//  * on each loop, whichever substate is set will be perfomed
//  * when the mode is changed by the Zephyr, FL_EXIT will automatically be set
//  * it is up to the FL_EXIT logic perform any actions for leaving flight mode
void StratoPIB::FlightMode()
{
    // todo: draw out flight mode state machine
    switch (inst_substate) {
    case FL_ENTRY:
        // perform setup
        log_nominal("Entering FL");
        inst_substate = FL_GPS_WAIT;
        break;
    case FL_GPS_WAIT:
        // wait for the first GPS message from Zephyr to set the time before moving on
        log_debug("Waiting on GPS time");
        if (true/*time_valid*/) { // FIXME: add back in GPS wait state
            inst_substate = (autonomous_mode) ? FLA_IDLE : FLM_IDLE;
        }
        break;
    case FL_ERROR_LANDING:
        // generic error state for flight mode to go to if any error is detected
        // this state can make sure the ground is informed, and go to the error loop to wait for ground intervention
        // before setting this substate, a ZephyrLogCrit should be sent
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        inst_substate = FL_ERROR_LOOP;
        break;
    case FL_ERROR_LOOP:
        // wait for ground
        // todo: add exit condition
        break;
    case FL_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in FL");
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        inst_substate = FL_SHUTDOWN_LOOP;
        break;
    case FL_SHUTDOWN_LOOP:
        break;
    case FL_EXIT:
        // perform cleanup
        log_nominal("Exiting FL");
        break;
    default:
        // we've made it here because we're in a mode-specific state
        if (autonomous_mode) {
            AutonomousFlight();
        } else {
            ManualFlight();
        }
        break;
    }
}

void StratoPIB::ManualFlight()
{
    switch (inst_substate) {
    case FLM_IDLE:
        log_debug("FL Manual Idle");
        if (CheckAction(COMMAND_REEL_IN)) {
            mcb_motion = MOTION_REEL_IN;
            inst_substate = FLM_SEND_RA;
            resend_attempted = false;
        } else if (CheckAction(COMMAND_REEL_OUT)) {
            mcb_motion = MOTION_REEL_OUT;
            inst_substate = FLM_SEND_RA;
            resend_attempted = false;
        } else if (CheckAction(COMMAND_DOCK)) {
            mcb_motion = MOTION_DOCK;
            inst_substate = FLM_SEND_RA;
            resend_attempted = false;
        }
        break;
    case FLM_SEND_RA:
        RA_ack_flag = NO_ACK;
        zephyrTX.RA();
        inst_substate = FLM_START_MOTION; //FIXME: should be FLM_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, 60);
        log_nominal("Sending RA");
        break;
    case FLM_WAIT_RAACK:
        if (ACK == RA_ack_flag) {
            inst_substate = FLM_START_MOTION;
            resend_attempted = false;
            log_nominal("RA ACK");
        } else if (NAK == RA_ack_flag) {
            inst_substate = FLM_IDLE;
            resend_attempted = false;
            ZephyrLogWarn("Cannot perform motion, RA NAK");
        } else if (CheckAction(RESEND_RA)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = FLM_SEND_RA;
            } else {
                resend_attempted = false;
                inst_substate = FLM_IDLE;
            }
        }
        break;
    case FLM_START_MOTION:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion commanded while motion ongoing");
            inst_substate = FL_ERROR_LANDING;
        }

        if (StartMCBMotion()) {
            inst_substate = FLM_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, 30);
        } else {
            ZephyrLogWarn("Motion start error");
            inst_substate = FL_ERROR_LANDING;
        }
        break;
    case FLM_VERIFY_MOTION:
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            inst_substate = FLM_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = FLM_START_MOTION;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                inst_substate = FL_ERROR_LANDING;
            }
        }
        break;
    case FLM_MONITOR_MOTION:
        // todo: what should be monitored? Just check for MCB messages?

        if (CheckAction(COMMAND_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            inst_substate = FLM_IDLE;
            break;
        }

        if (!mcb_motion_ongoing) {
            ZephyrLogFine("Motion complete");
            // todo: log results
            inst_substate = FLM_IDLE;
        }
        break;
    default:
        ZephyrLogWarn("Unknown manual substate");
        inst_substate = FLM_IDLE;
        break;
    }
}

void StratoPIB::AutonomousFlight()
{
    switch (inst_substate) {
    case FLA_IDLE:
        log_debug("FL autonomous idle");
        break;
    default:
        ZephyrLogWarn("Unknown autonomous substate");
        inst_substate = FLA_IDLE;
        break;
    }
}