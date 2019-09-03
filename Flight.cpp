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
    FLA_WAIT_PROFILE,
    FLA_SEND_RA,
    FLA_WAIT_RAACK,
    // FLA_MC_WARMUP,
    FLA_CONFIGURE_PU,
    FLA_UNDOCK,
    FLA_REEL_OUT,
    FLA_REEL_IN,
    FLA_DOCK,
    FLA_START_MOTION,
    FLA_VERIFY_UNDOCK,
    FLA_VERIFY_DOCK,
    FLA_VERIFY_MOTION,
    FLA_MONITOR_MOTION,
    FLA_DWELL,
    FLA_MCB_LP_WAIT,
    FLA_PU_DOWNLOAD,

    // general off-nominal states
    FL_ERROR_LOOP,
    FL_SHUTDOWN_LOOP,

    FL_ERROR_LANDING = MODE_ERROR,
    FL_SHUTDOWN_LANDING = MODE_SHUTDOWN,
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
        if (time_valid) {
            inst_substate = (autonomous_mode) ? FLA_IDLE : FLM_IDLE;
        }
        break;
    case FL_ERROR_LANDING:
        log_error("Landed in flight error");
        mcb_motion_ongoing = false;
        profiles_remaining = 0;
        mcb_motion = NO_MOTION;
        resend_attempted = false;
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        inst_substate = FL_ERROR_LOOP;
        break;
    case FL_ERROR_LOOP:
        log_debug("FL error loop");
        if (!mcb_low_power && CheckAction(RESEND_MCB_LP)) {
            scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
            mcbComm.TX_ASCII(MCB_GO_LOW_POWER); // just constantly send
        }

        if (CheckAction(EXIT_ERROR_STATE)) {
            log_nominal("Leaving flight error loop");
            inst_substate = FL_ENTRY;
        }
        break;
    case FL_SHUTDOWN_LANDING:
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
        inst_substate = FLM_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, ZEPHYR_RESEND_TIMEOUT);
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
                ZephyrLogWarn("Never received RAAck");
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
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
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

        // if we've reached the right profile trigger, schedule the night's profiles
        // TODO: fix SZA - will just immediately repeat profiles if Sun still meets limit
        if ((pib_config.sza_trigger && zephyrRX.zephyr_gps.solar_zenith_angle > pib_config.sza_minimum) ||
            (!pib_config.sza_trigger && (uint32_t) now() >= pib_config.time_trigger)) {
            if (ScheduleProfiles()) {
                ZephyrLogFine("Starting profiles for the night");
                inst_substate = FLA_WAIT_PROFILE;
            } else {
                ZephyrLogWarn("Error scheduling autonomous profiles");
                inst_substate = FL_ERROR_LANDING;
            }
        }
        break;
    case FLA_WAIT_PROFILE:
        log_debug("FLA wait profile");
        if (0 == profiles_remaining) {
            log_nominal("Finished with profiles for the night");
            inst_substate = FLA_IDLE;
        }

        if (CheckAction(COMMAND_BEGIN_PROFILE)) {
            log_nominal("Time for scheduled profile");
            profiles_remaining--;
            inst_substate = FLA_SEND_RA;
        }
        break;
    case FLA_SEND_RA:
        RA_ack_flag = NO_ACK;
        zephyrTX.RA();
        inst_substate = FLA_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, ZEPHYR_RESEND_TIMEOUT);
        log_nominal("Sending RA");
        break;
    case FLA_WAIT_RAACK:
        log_debug("FLA wait RA Ack");
        if (ACK == RA_ack_flag) {
            inst_substate = FLA_CONFIGURE_PU;
            resend_attempted = false;
            log_nominal("RA ACK");
        } else if (NAK == RA_ack_flag) {
            inst_substate = FLA_WAIT_PROFILE; // todo, if NAK'ed, just abort profile and wait for the next?
            resend_attempted = false;
            ZephyrLogWarn("Cannot perform motion, RA NAK");
        } else if (CheckAction(RESEND_RA)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = FLA_SEND_RA;
            } else {
                ZephyrLogWarn("Never received RAAck");
                resend_attempted = false;
                inst_substate = FLA_WAIT_PROFILE;
            }
        }
        break;
    // case FLA_MC_WARMUP:
    case FLA_CONFIGURE_PU:
        log_debug("FLA configure PU");
        // todo: this
        inst_substate = FLA_REEL_OUT;
        break;
    case FLA_UNDOCK:
        log_debug("FLA undock");
        // todo: is this state needed?
        inst_substate = FL_ERROR_LANDING;
        break;
    case FLA_REEL_OUT:
        log_debug("FLA reel out");
        mcb_motion = MOTION_REEL_OUT;
        deploy_length = pib_config.profile_size;
        inst_substate = FLA_START_MOTION;
        resend_attempted = false;
        break;
    case FLA_REEL_IN:
        log_debug("FLA reel in");
        mcb_motion = MOTION_REEL_IN;
        retract_length = pib_config.profile_size - pib_config.dock_amount;
        inst_substate = FLA_START_MOTION;
        resend_attempted = false;
        break;
    case FLA_DOCK:
        log_debug("FLA dock");
        mcb_motion = MOTION_DOCK;
        dock_length = pib_config.dock_amount + pib_config.dock_overshoot;
        inst_substate = FLA_START_MOTION;
        resend_attempted = false;
        break;
    case FLA_VERIFY_UNDOCK:
        log_debug("FLA verify undock");
        // todo: is this needed?
        inst_substate = FL_ERROR_LANDING;
        break;
    case FLA_VERIFY_DOCK:
        log_debug("FLA verify dock");
        // todo: this
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
        inst_substate = FLA_MCB_LP_WAIT;
        break;
    case FLA_START_MOTION:
        log_debug("FLA start motion");
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion commanded while motion ongoing");
            inst_substate = FL_ERROR_LANDING;
        }

        if (StartMCBMotion()) {
            inst_substate = FLA_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
        } else {
            ZephyrLogWarn("Motion start error");
            inst_substate = FL_ERROR_LANDING;
        }
        break;
    case FLA_VERIFY_MOTION:
        log_debug("FLA verify motion");
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
            inst_substate = FLA_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                inst_substate = FLA_START_MOTION;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                inst_substate = FL_ERROR_LANDING;
            }
        }
        break;
    case FLA_MONITOR_MOTION:
        log_debug("FLA monitor motion");
        // todo: what should be monitored? Just check for MCB messages?

        if (CheckAction(COMMAND_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogWarn("Commanded motion stop in autonomous");
            inst_substate = FL_ERROR_LANDING;
            break;
        }

        if (!mcb_motion_ongoing) {
            log_nominal("Motion complete");
            switch (mcb_motion) {
            case MOTION_REEL_OUT:
                if (scheduler.AddAction(COMMAND_END_DWELL, pib_config.dwell_time)) {
                    snprintf(log_array, LOG_ARRAY_SIZE, "Scheduled dwell: %u s", pib_config.dwell_time);
                    log_nominal(log_array);
                    inst_substate = FLA_DWELL;
                } else {
                    ZephyrLogCrit("Unable to schedule dwell");
                    inst_substate = FL_ERROR_LANDING;
                }
                break;
            case MOTION_REEL_IN:
                inst_substate = FLA_DOCK;
                break;
            case MOTION_DOCK:
                inst_substate = FLA_VERIFY_DOCK;
                break;
            default:
                ZephyrLogWarn("Unknown motion finished in autonomous monitor");
                inst_substate = FL_ERROR_LANDING;
                break;
            }
        }
        break;
    case FLA_DWELL:
        log_debug("FLA dwell");
        if (CheckAction(COMMAND_END_DWELL)) {
            log_nominal("Finished dwell");
            inst_substate = FLA_REEL_IN;
        }
        break;
    case FLA_MCB_LP_WAIT:
        if (mcb_low_power) {
            log_nominal("MCB in low power after profile");
            mcb_low_power = false;
            inst_substate = FLA_PU_DOWNLOAD;
        } else if (CheckAction(RESEND_MCB_LP)) {
            if (!resend_attempted) {
                resend_attempted = true;
                mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never powered off after profile");
                inst_substate = FL_ERROR_LANDING;
            }
        }
    case FLA_PU_DOWNLOAD:
        log_debug("FLA PU download");
        snprintf(log_array, LOG_ARRAY_SIZE, "Docked after full profile of %f revs", pib_config.profile_size);
        ZephyrLogFine(log_array);
        // todo: actual PU code
        inst_substate = FLA_WAIT_PROFILE;
        break;
    default:
        ZephyrLogWarn("Unknown autonomous substate");
        inst_substate = FLA_IDLE;
        break;
    }
}