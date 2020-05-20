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
    FLM_CHECK_PU,
    FLM_MANUAL_MOTION,
    FLM_REDOCK,
    FLM_TSEN,
    FLM_PU_OFFLOAD,
    FLM_PROFILE,

    // autonomous
    FLA_IDLE,
    FLA_WAIT_PROFILE,
    FLA_TSEN,
    FLA_PROFILE,
    FLA_PU_OFFLOAD,
    FLA_NOTE_PROFILE_END,

    // general off-nominal states
    FL_ERROR_LOOP,
    FL_SHUTDOWN_LOOP,

    // StratoCore-specified states
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
            if (!ScheduleNextTSEN()) {
                ZephyrLogCrit("Unable to schedule next TSEN read");
                inst_substate = FL_ERROR_LANDING;
            }
        }
        break;
    case FL_ERROR_LANDING:
        log_error("Landed in flight error");
        scheduler.ClearSchedule();
        mcb_motion_ongoing = false;
        profiles_remaining = 0;
        mcb_motion = NO_MOTION;
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
        mcb_low_power = false;
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
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
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
        if (CheckAction(ACTION_REEL_IN)) {
            log_nominal("Reel in manual command");
            mcb_motion = MOTION_REEL_IN;
            Flight_ManualMotion(true);
            inst_substate = FLM_MANUAL_MOTION;
        } else if (CheckAction(ACTION_REEL_OUT)) {
            log_nominal("Reel out manual command");
            mcb_motion = MOTION_REEL_OUT;
            Flight_ManualMotion(true);
            inst_substate = FLM_MANUAL_MOTION;
        } else if (CheckAction(ACTION_DOCK)) {
            log_nominal("Dock manual command");
            mcb_motion = MOTION_DOCK;
            Flight_ManualMotion(true);
            inst_substate = FLM_MANUAL_MOTION;
        } else if (CheckAction(ACTION_CHECK_PU)) {
            log_nominal("Check PU manual command");
            Flight_CheckPU(true);
            inst_substate = FLM_CHECK_PU;
        } else if (CheckAction(COMMAND_REDOCK)) {
            log_nominal("Redock manual command");
            mcb_motion = MOTION_IN_NO_LW;
            Flight_ReDock(true);
            inst_substate = FLM_REDOCK;
        } else if (CheckAction(COMMAND_SEND_TSEN)) {
            log_nominal("Send TSEN manual command");
            Flight_TSEN(true);
            inst_substate = FLM_TSEN;
        } else if (CheckAction(COMMAND_MANUAL_PROFILE)) {
            log_nominal("Profile manual command");
            Flight_Profile(true);
            inst_substate = FLM_PROFILE;
        } else if (CheckAction(ACTION_OFFLOAD_PU)) {
            log_nominal("Offload PU Manual");
            Flight_PUOffload(true);
            inst_substate = FLM_PU_OFFLOAD;
        }
        break;

    case FLM_CHECK_PU:
        if (Flight_CheckPU(false)) {
            // only send status if the PU check succeeded (otherwise an error message will have been sent)
            if (check_pu_success) {
                snprintf(log_array, LOG_ARRAY_SIZE, "PU status: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", pu_status.time, pu_status.v_battery, pu_status.i_charge, pu_status.therm1, pu_status.therm2, pu_status.heater_stat);
                ZephyrLogFine(log_array);
            }
            inst_substate = FLM_IDLE;
        }
        break;

    case FLM_MANUAL_MOTION:
        if (Flight_ManualMotion(false)) {
            inst_substate = FLM_IDLE;
        }
        break;

    case FLM_REDOCK:
        if (Flight_ReDock(false)) {
            inst_substate = FLM_IDLE;
        }
        break;

    case FLM_TSEN:
        if (Flight_TSEN(false)) {
            inst_substate = FLM_IDLE;
            if (!ScheduleNextTSEN()) {
                ZephyrLogCrit("Unable to schedule next TSEN read");
                inst_substate = FL_ERROR_LANDING;
            }
        }
        break;

    case FLM_PU_OFFLOAD:
        if (Flight_PUOffload(false)) {
            inst_substate = FLM_IDLE;
        }
        break;

    case FLM_PROFILE:
        if (Flight_Profile(false)) {
            inst_substate = FLM_IDLE;
            if (!ScheduleNextTSEN()) {
                ZephyrLogCrit("Unable to schedule next TSEN read");
                inst_substate = FL_ERROR_LANDING;
            }
        }
        break;

    default:
        log_error("Unknown manual substate");
        break;
    };
}

void StratoPIB::AutonomousFlight()
{
    switch (inst_substate) {
    case FLA_IDLE:
        if (zephyrRX.zephyr_gps.solar_zenith_angle < 45) {
            profiles_remaining = pibConfigs.num_profiles.Read();
            profiles_scheduled = false;
        } else if (0 != profiles_remaining && pibConfigs.sza_trigger.Read() && zephyrRX.zephyr_gps.solar_zenith_angle > pibConfigs.sza_minimum.Read()) {
            if (profiles_scheduled) {
                inst_substate = FLA_WAIT_PROFILE;
            } else if (ScheduleProfiles()) { // Schedule Profiles sends result as TM
                profiles_scheduled = true;
                inst_substate = FLA_WAIT_PROFILE;
            } else {
                inst_substate = FL_ERROR_LANDING;
            }
        } else if (0 != profiles_remaining && !pibConfigs.sza_trigger.Read() && (uint32_t) now() >= pibConfigs.time_trigger.Read()) {
            if (profiles_scheduled) {
                inst_substate = FLA_WAIT_PROFILE;
            } else if (ScheduleProfiles()) { // Schedule Profiles sends result as TM
                profiles_scheduled = true;
                inst_substate = FLA_WAIT_PROFILE;
            } else {
                inst_substate = FL_ERROR_LANDING;
            }
        } else if (CheckAction(COMMAND_SEND_TSEN)) {
            Flight_TSEN(true);
            inst_substate = FLA_TSEN;
        }
        break;

    case FLA_WAIT_PROFILE:
        if (CheckAction(ACTION_BEGIN_PROFILE)) {
            Flight_Profile(true);
            inst_substate = FLA_PROFILE;
        } else if (CheckAction(COMMAND_SEND_TSEN)) {
            Flight_TSEN(true);
            inst_substate = FLA_TSEN;
        }
        break;

    case FLA_TSEN:
        if (Flight_TSEN(false)) {
            inst_substate = FLA_IDLE;
            if (!ScheduleNextTSEN()) {
                ZephyrLogCrit("Unable to schedule next TSEN read");
                inst_substate = FL_ERROR_LANDING;
            }
        }
        break;

    case FLA_PROFILE:
        if (Flight_Profile(false)) {
            Flight_PUOffload(true);
            inst_substate = FLA_PU_OFFLOAD;
        }
        break;

    case FLA_PU_OFFLOAD:
        if (Flight_PUOffload(false)) {
            inst_substate = FLA_NOTE_PROFILE_END;
        }
        break;

    case FLA_NOTE_PROFILE_END:
        if (profiles_remaining != 0) profiles_remaining--;

        if (!ScheduleNextTSEN()) {
            ZephyrLogCrit("Unable to schedule next TSEN read");
            inst_substate = FL_ERROR_LANDING;
        } else {
            inst_substate = FLA_IDLE;
        }
        break;

    default:
        log_error("Unknown autonomous substate");
        break;
    };
}