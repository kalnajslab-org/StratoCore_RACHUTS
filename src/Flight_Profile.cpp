/*
 *  Flight_Profile.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoRatchuts.h"

enum ProfileStates_t {
    ST_ENTRY,
    ST_SEND_RA,
    ST_WAIT_RAACK,
    ST_SET_PU_PROFILE,
    ST_CONFIRM_PU_PROFILE,
    ST_PREPROFILE_WAIT,
    ST_REEL_OUT,
    ST_DWELL,
    ST_REEL_IN,
    ST_DOCK_WAIT,
    ST_DOCK,
    ST_GET_PU_STATUS,
    ST_VERIFY_DOCK,
    ST_REDOCK,
    ST_START_MOTION,
    ST_VERIFY_MOTION,
    ST_MONITOR_MOTION,
    ST_CONFIRM_MCB_LP,
};

static ProfileStates_t profile_state = ST_ENTRY;
static bool resend_attempted = false;
static uint8_t redock_count = 0;

bool StratoRatchuts::Flight_Profile(bool restart_state)
{
    if (restart_state) profile_state = ST_ENTRY;

    switch (profile_state) {
    case ST_ENTRY:
    case ST_SEND_RA:
        RA_ack_flag = NO_ACK;
        ZephyrTXpoke(ZEPHYRTX_RA);
        profile_state = ST_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, ZEPHYR_RESEND_TIMEOUT);
        log_nominal("Sending RA");
        break;

    case ST_WAIT_RAACK:
        log_debug("FLA wait RA Ack");
        if (ACK == RA_ack_flag) { // set by Zephyr RA ack handler
            profile_state = ST_SET_PU_PROFILE;
            resend_attempted = false;
            log_nominal("RA ACK");
        } else if (NAK == RA_ack_flag) {
            ZephyrLogWarn("Cannot perform motion, RA NAK");
            resend_attempted = false;
            return true;
        } else if (CheckAction(RESEND_RA)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_SEND_RA;
            } else {
                ZephyrLogWarn("Never received RAAck");
                resend_attempted = false;
                return true;
            }
        }
        break;

    case ST_SET_PU_PROFILE:
        retract_length = pibConfigs.profile_size.Read() - pibConfigs.dock_amount.Read();
        deploy_length = pibConfigs.profile_size.Read();
        dock_length = pibConfigs.dock_amount.Read() + pibConfigs.dock_overshoot.Read();
        // pu_measure will be set in PURouter when the RPU acks the go-measure command
        pu_measure = false; 
        resend_attempted = false;
        // Send the profile command to the PU with the configured parameters
        PUStartProfile();
        scheduler.AddAction(RESEND_PU_GOPROFILE, PU_RESEND_TIMEOUT);
        profile_state = ST_CONFIRM_PU_PROFILE;
        break;

    case ST_CONFIRM_PU_PROFILE:
        if (pu_measure) { // set in PURouter when RPU acks RPU_GO_MEASURE
            profile_state = ST_PREPROFILE_WAIT;
            scheduler.AddAction(ACTION_END_PREPROFILE, pibConfigs.preprofile_time.Read());
        } else if (CheckAction(RESEND_PU_GOPROFILE)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_SET_PU_PROFILE;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("RPU not responding to go-measure command");
                return true;
            }
        }
        break;

    case ST_PREPROFILE_WAIT:
        if (CheckAction(ACTION_END_PREPROFILE)) {
            profile_state = ST_REEL_OUT;
            resend_attempted = false;
        }
        break;

    case ST_REEL_OUT:
        log_debug("FLA reel out");
        mcb_motion = MOTION_REEL_OUT;
        profile_state = ST_START_MOTION;
        resend_attempted = false;
        break;

    case ST_REEL_IN:
        log_debug("FLA reel in");
        mcb_motion = MOTION_REEL_IN;
        profile_state = ST_START_MOTION;
        resend_attempted = false;
        break;

    case ST_DOCK_WAIT:
        // wait for the timeout set for the reel out or the backup action, whichever comes first
        if (CheckAction(ACTION_MOTION_TIMEOUT) || CheckAction(ACTION_END_DOCK_WAIT)) {
            profile_state = ST_DOCK;
        }
        break;

    case ST_DOCK:
        log_debug("FLA dock");
        mcb_motion = MOTION_DOCK;
        profile_state = ST_START_MOTION;
        resend_attempted = false;
        break;

    case ST_GET_PU_STATUS:
        if (Flight_CheckPU(false)) {
            profile_state = ST_VERIFY_DOCK;
        }
        break;

    case ST_VERIFY_DOCK:
        if (pibConfigs.pu_docked.Read()) { // written by Flight_CheckPU from RPU status
            mcbComm.TX_ASCII(MCB_ZERO_REEL);
            delay(100);
            mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
            scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
            profile_state = ST_CONFIRM_MCB_LP;
        } else {
            if ((pibConfigs.num_redock.Read() + 1) == ++redock_count) {
                ZephyrLogCrit("No dock! Exceeded allowable number of redock attempts");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            } else {
                deploy_length = pibConfigs.redock_out.Read();
                retract_length = pibConfigs.redock_in.Read();
                Flight_ReDock(true);
                profile_state = ST_REDOCK;
            }
        }
        break;

    case ST_REDOCK:
        if (Flight_ReDock(false)) {
            Flight_CheckPU(true);
            profile_state = ST_GET_PU_STATUS;
        }
        break;

    case ST_START_MOTION:
        log_debug("FLA start motion");
        if (mcb_motion_ongoing) { // set in MCBRouter when MCB acks motion command
            ZephyrLogWarn("Motion commanded while motion ongoing");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
        }

        if (StartMCBMotion()) {
            profile_state = ST_VERIFY_MOTION;
            scheduler.AddAction(RESEND_MOTION_COMMAND, MCB_RESEND_TIMEOUT);
        } else {
            ZephyrLogWarn("Motion start error");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
        }
        break;

    case ST_VERIFY_MOTION:
        log_debug("FLA verify motion");
        if (mcb_motion_ongoing) { // set in MCBRouter when MCB acks motion command
            log_nominal("MCB commanded motion");
            scheduler.AddAction(ACTION_MOTION_TIMEOUT, max_profile_seconds);
            profile_state = ST_MONITOR_MOTION;
        }

        if (CheckAction(RESEND_MOTION_COMMAND)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_START_MOTION;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never confirmed motion");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            }
        }
        break;

    case ST_MONITOR_MOTION:
        log_debug("FLA monitor motion");

        if (CheckAction(ACTION_MOTION_STOP)) {
            ZephyrLogWarn("Commanded motion stop in autonomous");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            break;
        }

        if (CheckAction(ACTION_MOTION_TIMEOUT)) {
            SendMCBTM(CRIT, "MCB Motion took longer than expected");
            mcbComm.TX_ASCII(MCB_CANCEL_MOTION);
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            break;
        }

        if (!mcb_motion_ongoing) { // cleared in MCBRouter when MCB reports motion complete
            log_nominal("Motion complete");
            switch (mcb_motion) {
            case MOTION_REEL_OUT:
                SendMCBTM(FINE, "Finished profile reel out");
                if (scheduler.AddAction(ACTION_END_DWELL, pibConfigs.dwell_time.Read())) {
                    snprintf(log_array, LOG_ARRAY_SIZE, "Scheduled dwell: %u s", pibConfigs.dwell_time.Read());
                    log_nominal(log_array);
                    profile_state = ST_DWELL;
                } else {
                    ZephyrLogCrit("Unable to schedule dwell");
                    inst_substate = MODE_ERROR; // will force exit of Flight_Profile
                }
                break;
            case MOTION_REEL_IN:
                SendMCBTM(FINE, "Finished profile reel in");
                scheduler.AddAction(ACTION_END_DOCK_WAIT, 60);
                profile_state = ST_DOCK_WAIT;
                break;
            case MOTION_DOCK:
                // MCB TM sent in MCBRouter handler for MCB_MOTION_FAULT
                redock_count = 0;
                Flight_CheckPU(true); // start checking the PU
                profile_state = ST_GET_PU_STATUS;
                break;
            default:
                SendMCBTM(CRIT, "Unknown motion finished in profile monitor");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
                break;
            }
        }
        break;

    case ST_DWELL:
        log_debug("FLA dwell");
        if (CheckAction(ACTION_END_DWELL)) {
            log_nominal("Finished dwell");
            profile_state = ST_REEL_IN;
        }
        break;

    case ST_CONFIRM_MCB_LP:
        if (mcb_low_power) { // set in MCBRouter when MCB acks MCB_GO_LOW_POWER
            log_nominal("Profile finished, MCB in low power");
            mcb_low_power = false;
            return true;
        } else if (CheckAction(RESEND_MCB_LP)) {
            if (!resend_attempted) {
                resend_attempted = true;
                mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
            } else {
                resend_attempted = false;
                ZephyrLogWarn("MCB never powered off after profile");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            }
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}