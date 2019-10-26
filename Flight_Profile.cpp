/*
 *  Flight_Profile.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoPIB.h"

enum ProfileStates_t {
    ST_ENTRY,
    ST_SEND_RA,
    ST_WAIT_RAACK,
    ST_HOUSKEEPING_CHECK,
    ST_SET_PU_WARMUP,
    ST_CONFIRM_PU_WARMUP,
    ST_WARMUP,
    ST_GET_TSEN,
    ST_SET_PU_PROFILE,
    ST_CONFIRM_PU_PROFILE,
    ST_PREPROFILE_WAIT,
    ST_REEL_OUT,
    ST_DWELL,
    ST_REEL_IN,
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

bool StratoPIB::Flight_Profile(bool restart_state)
{
    if (restart_state) profile_state = ST_ENTRY;

    switch (profile_state) {
    case ST_ENTRY:
    case ST_SEND_RA:
        RA_ack_flag = NO_ACK;
        zephyrTX.RA();
        profile_state = ST_WAIT_RAACK;
        scheduler.AddAction(RESEND_RA, ZEPHYR_RESEND_TIMEOUT);
        log_nominal("Sending RA");
        break;

    case ST_WAIT_RAACK:
        log_debug("FLA wait RA Ack");
        if (ACK == RA_ack_flag) {
            profile_state = ST_HOUSKEEPING_CHECK;
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

    case ST_HOUSKEEPING_CHECK:
        profile_state = ST_SET_PU_WARMUP;
        resend_attempted = false;
        break;

    case ST_SET_PU_WARMUP:
        pu_warmup = false;
        puComm.TX_WarmUp(-20.0f,0.0f,-15.0f,1,1); // TODO: get proper parameters
        scheduler.AddAction(RESEND_PU_WARMUP, PU_RESEND_TIMEOUT);
        profile_state = ST_CONFIRM_PU_WARMUP;
        break;

    case ST_CONFIRM_PU_WARMUP:
        if (pu_warmup) {
            profile_state = ST_WARMUP;
            scheduler.AddAction(ACTION_END_WARMUP, WARMUP_PERIOD);
        } else if (CheckAction(RESEND_PU_WARMUP)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_SET_PU_WARMUP;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not responding to warmup command");
                return true;
            }
        }
        break;

    case ST_WARMUP:
        if (CheckAction(ACTION_END_WARMUP)) {
            Flight_TSEN(true);
            profile_state = ST_GET_TSEN;
        }
        break;

    case ST_GET_TSEN:
        if (Flight_TSEN(false)) {
            profile_state = ST_SET_PU_PROFILE;
        }
        break;

    case ST_SET_PU_PROFILE:
        pu_profile = false;
        PUStartProfile();
        scheduler.AddAction(RESEND_PU_GOPROFILE, PU_RESEND_TIMEOUT);
        profile_state = ST_CONFIRM_PU_PROFILE;
        break;

    case ST_CONFIRM_PU_PROFILE:
        if (pu_profile) {
            profile_state = ST_PREPROFILE_WAIT;
            scheduler.AddAction(ACTION_END_PREPROFILE, PREPROFILE_PERIOD);
        } else if (CheckAction(RESEND_PU_GOPROFILE)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_SET_PU_PROFILE;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not responding to profile command");
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
        deploy_length = pib_config.profile_size;
        profile_state = ST_START_MOTION;
        resend_attempted = false;
        break;

    case ST_REEL_IN:
        log_debug("FLA reel in");
        mcb_motion = MOTION_REEL_IN;
        retract_length = pib_config.profile_size - pib_config.dock_amount;
        profile_state = ST_START_MOTION;
        resend_attempted = false;
        break;

    case ST_DOCK:
        log_debug("FLA dock");
        mcb_motion = MOTION_DOCK;
        dock_length = pib_config.dock_amount + pib_config.dock_overshoot;
        profile_state = ST_START_MOTION;
        resend_attempted = false;
        break;

    case ST_GET_PU_STATUS:
        if (Flight_CheckPU(false)) {
            profile_state = ST_VERIFY_DOCK;
        }
        break;

    case ST_VERIFY_DOCK:
        if (pib_config.pu_docked) {
            mcbComm.TX_ASCII(MCB_ZERO_REEL);
            delay(100);
            mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
            scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
            profile_state = ST_CONFIRM_MCB_LP;
        } else {
            if (3 == ++redock_count) { // TODO: configurable? Don't hard-code
                ZephyrLogCrit("Two redock failures");
                inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            } else {
                deploy_length = 5; // TODO: don't hard-code these
                retract_length = 10;
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
        if (mcb_motion_ongoing) {
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
        if (mcb_motion_ongoing) { // set in the Ack handler
            log_nominal("MCB commanded motion");
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
            // todo: verification of motion stop
            ZephyrLogWarn("Commanded motion stop in autonomous");
            inst_substate = MODE_ERROR; // will force exit of Flight_Profile
            break;
        }

        if (!mcb_motion_ongoing) {
            log_nominal("Motion complete");
            switch (mcb_motion) {
            case MOTION_REEL_OUT:
                SendMCBTM(FINE, "Finished autonomous reel out");
                if (scheduler.AddAction(ACTION_END_DWELL, pib_config.dwell_time)) {
                    snprintf(log_array, LOG_ARRAY_SIZE, "Scheduled dwell: %u s", pib_config.dwell_time);
                    log_nominal(log_array);
                    profile_state = ST_DWELL;
                } else {
                    ZephyrLogCrit("Unable to schedule dwell");
                    inst_substate = MODE_ERROR; // will force exit of Flight_Profile
                }
                break;
            case MOTION_REEL_IN:
                SendMCBTM(FINE, "Finished autonomous reel in");
                profile_state = ST_DOCK;
                break;
            case MOTION_DOCK:
                SendMCBTM(FINE, "Finished autonomous dock");
                redock_count = 0;
                Flight_CheckPU(true); // start checking the PU
                profile_state = ST_GET_PU_STATUS;
                break;
            default:
                SendMCBTM(CRIT, "Unknown motion finished in autonomous monitor");
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
        if (mcb_low_power) {
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