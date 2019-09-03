/*
 *  StratoPIB.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file implements an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the RACHuTS Profiler Interface Board, or PIB.
 */

#include "StratoPIB.h"

StratoPIB::StratoPIB()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
    , mcbComm(&MCB_SERIAL)
{
}

void StratoPIB::InstrumentSetup()
{
    // for RS232 transceiver
    pinMode(FORCEOFF_232, OUTPUT);
    pinMode(FORCEON_232, OUTPUT);
    digitalWrite(FORCEOFF_232, HIGH);
    digitalWrite(FORCEON_232, HIGH);

    // safe pin required by Zephyr
    pinMode(SAFE_PIN, OUTPUT);
    digitalWrite(SAFE_PIN, LOW);

    if (!pibStorage.LoadFromEEPROM()) {
        ZephyrLogWarn("EEPROM updated");
    }
}

void StratoPIB::InstrumentLoop()
{
    WatchFlags();
}

// The telecommand handler must return ACK/NAK
bool StratoPIB::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";

    switch (telecommand) {
    case DEPLOYx:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        deploy_length = mcbParam.deployLen;
        SetAction(COMMAND_REEL_OUT); // will be ignored if wrong mode
        break;
    case DEPLOYv:
        if (EEPROM_UPDATE_FLOAT(pibStorage, deploy_velocity, mcbParam.deployVel)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set deploy_velocity: %f", pib_config.deploy_velocity);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting deploy_velocity: %f", pib_config.deploy_velocity);
            ZephyrLogWarn(log_array);
        }
        break;
    case DEPLOYa:
        mcbComm.TX_Out_Acc(mcbParam.deployAcc); // todo: verification + ack
        break;
    case RETRACTx:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        retract_length = mcbParam.retractLen;
        SetAction(COMMAND_REEL_IN); // will be ignored if wrong mode
        break;
    case RETRACTv:
        if (EEPROM_UPDATE_FLOAT(pibStorage, retract_velocity, mcbParam.retractVel)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set retract_velocity: %f", pib_config.retract_velocity);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting retract_velocity: %f", pib_config.retract_velocity);
            ZephyrLogWarn(log_array);
        }
        break;
    case RETRACTa:
        mcbComm.TX_In_Acc(mcbParam.retractAcc); // todo: verification + ack
        break;
    case DOCKx:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        dock_length = mcbParam.dockLen;
        SetAction(COMMAND_DOCK); // will be ignored if wrong mode
        break;
    case DOCKv:
        if (EEPROM_UPDATE_FLOAT(pibStorage, dock_velocity, mcbParam.dockVel)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set dock_velocity: %f", pib_config.dock_velocity);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting dock_velocity: %f", pib_config.dock_velocity);
            ZephyrLogWarn(log_array);
        }
        break;
    case DOCKa:
        mcbComm.TX_Dock_Acc(mcbParam.dockAcc); // todo: verification + ack
        break;
    case FULLRETRACT:
        // todo: determine implementation
        break;
    case CANCELMOTION:
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(COMMAND_MOTION_STOP);
        break;
    case SETAUTO:
        if (!mcb_motion_ongoing) {
            autonomous_mode = true;
            inst_substate = MODE_ENTRY; // restart FL in auto
            ZephyrLogFine("Set mode to auto");
        } else {
            ZephyrLogWarn("Motion ongoing, can't update mode");
            return false;
        }
        break;
    case SETMANUAL:
        if (!mcb_motion_ongoing) {
            autonomous_mode = false;
            inst_substate = MODE_ENTRY; // restart FL in manual
            ZephyrLogFine("Set mode to manual");
        } else {
            ZephyrLogWarn("Motion ongoing, can't update mode");
            return false;
        }
        break;
    case SETSZAMIN:
        if (EEPROM_UPDATE_FLOAT(pibStorage, sza_minimum, pibParam.szaMinimum)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set sza_minimum: %f", pib_config.sza_minimum);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting sza_minimum: %f", pib_config.sza_minimum);
            ZephyrLogWarn(log_array);
        }
        break;
    case SETPROFILESIZE:
        if (EEPROM_UPDATE_FLOAT(pibStorage, profile_size, pibParam.profileSize)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set profile_size: %f", pib_config.profile_size);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting profile_size: %f", pib_config.profile_size);
            ZephyrLogWarn(log_array);
        }
        break;
    case SETDOCKAMOUNT:
        if (EEPROM_UPDATE_FLOAT(pibStorage, dock_amount, pibParam.dockAmount)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set dock_amount: %f", pib_config.dock_amount);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting dock_amount: %f", pib_config.dock_amount);
            ZephyrLogWarn(log_array);
        }
        break;
    case SETDWELLTIME:
        if (EEPROM_UPDATE_UINT16(pibStorage, dwell_time, pibParam.dwellTime)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set dwell_time: %u", pib_config.dwell_time);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting dwell_time: %u", pib_config.dwell_time);
            ZephyrLogWarn(log_array);
        }
        break;
    case SETPROFILEPERIOD:
        if (EEPROM_UPDATE_UINT16(pibStorage, profile_period, pibParam.profilePeriod)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set profile_period: %u", pib_config.profile_period);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting profile_period: %u", pib_config.profile_period);
            ZephyrLogWarn(log_array);
        }
        break;
    case SETNUMPROFILES:
        if (EEPROM_UPDATE_UINT8(pibStorage, num_profiles, pibParam.numProfiles)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set num_profiles: %u", pib_config.num_profiles);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting num_profiles: %u", pib_config.num_profiles);
            ZephyrLogWarn(log_array);
        }
        break;
    case SETTIMETRIGGER:
        if ((uint32_t) now() > pibParam.timeTrigger) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Can't use time trigger in past: %lu is less than %lu", pibParam.timeTrigger, (uint32_t) now());
            ZephyrLogWarn(log_array);
            break;
        }
        if (EEPROM_UPDATE_UINT32(pibStorage, time_trigger, pibParam.timeTrigger)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set time_trigger: %lu", pib_config.time_trigger);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting time_trigger: %lu", pib_config.time_trigger);
            ZephyrLogWarn(log_array);
        }
        break;
    case USESZATRIGGER:
        if (EEPROM_UPDATE_BOOL(pibStorage, sza_trigger, true)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set sza_trigger: %u", pib_config.sza_trigger);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting sza_trigger: %u", pib_config.sza_trigger);
            ZephyrLogWarn(log_array);
        }
        break;
    case USETIMETRIGGER:
        if (EEPROM_UPDATE_BOOL(pibStorage, sza_trigger, false)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set sza_trigger: %u", pib_config.sza_trigger);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting sza_trigger: %u", pib_config.sza_trigger);
            ZephyrLogWarn(log_array);
        }
        break;
    case SETDOCKOVERSHOOT:
        if (EEPROM_UPDATE_FLOAT(pibStorage, dock_overshoot, pibParam.dockOvershoot)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Set dock_overshoot: %f", pib_config.dock_overshoot);
            ZephyrLogFine(log_array);
        } else {
            snprintf(log_array, LOG_ARRAY_SIZE, "Error setting dock_overshoot: %f", pib_config.dock_overshoot);
            ZephyrLogWarn(log_array);
        }
        break;
    case EXITERROR:
        SetAction(EXIT_ERROR_STATE);
        break;
    default:
        log_error("Unknown TC received");
        // error case here
        break;
    }
    return true;
}

void StratoPIB::ActionHandler(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return;
    }

    // set the flag and reset the stale count
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

bool StratoPIB::CheckAction(uint8_t action)
{
    // for safety, ensure index doesn't exceed array size
    if (action >= NUM_ACTIONS) {
        log_error("Out of bounds action flag access");
        return false;
    }

    // check and clear the flag if it is set, return the value
    if (action_flags[action].flag_value) {
        action_flags[action].flag_value = false;
        action_flags[action].stale_count = 0;
        return true;
    } else {
        return false;
    }
}

void StratoPIB::SetAction(uint8_t action)
{
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

void StratoPIB::WatchFlags()
{
    // monitor for and clear stale flags
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (action_flags[i].flag_value) {
            action_flags[i].stale_count++;
            if (action_flags[i].stale_count >= FLAG_STALE) {
                action_flags[i].flag_value = false;
                action_flags[i].stale_count = 0;
            }
        }
    }
}

void StratoPIB::RunMCBRouter()
{
    SerialMessage_t rx_msg = mcbComm.RX();

    while (NO_MESSAGE != rx_msg) {
        if (ASCII_MESSAGE == rx_msg) {
            HandleMCBASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandleMCBAck();
        } else {
            log_error("Non-ASCII message from MCB");
        }

        rx_msg = mcbComm.RX();
    }
}

void StratoPIB::HandleMCBASCII()
{
    switch (mcbComm.ascii_rx.msg_id) {
    case MCB_MOTION_FINISHED:
        log_nominal("MCB motion finished");
        mcb_motion_ongoing = false;
        break;
    case MCB_ERROR:
        if (mcbComm.RX_Error(log_array, LOG_ARRAY_SIZE)) {
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    case MCB_MOTION_FAULT:
        if (mcbComm.RX_Motion_Fault(motion_fault, motion_fault+1, motion_fault+2, motion_fault+3,
                                    motion_fault+4, motion_fault+5, motion_fault+6, motion_fault+7)) {
            mcb_motion_ongoing = false;
            snprintf(log_array, LOG_ARRAY_SIZE, "MCB Fault: %u,%u,%u,%u,%u,%u,%u,%u", motion_fault[0], motion_fault[1],
                     motion_fault[2], motion_fault[3], motion_fault[4], motion_fault[5], motion_fault[6], motion_fault[7]);
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    default:
        log_error("Unknown MCB message received");
        break;
    }
}

void StratoPIB::HandleMCBAck()
{
    switch (mcbComm.ack_id) {
    case MCB_GO_LOW_POWER:
        log_nominal("MCB in low power");
        mcb_low_power = true;
        break;
    case MCB_REEL_IN:
        if (MOTION_REEL_IN == mcb_motion) mcb_motion_ongoing = true;
        break;
    case MCB_REEL_OUT:
        if (MOTION_REEL_OUT == mcb_motion) mcb_motion_ongoing = true;
        break;
    case MCB_DOCK:
        if (MOTION_DOCK == mcb_motion) mcb_motion_ongoing = true;
        break;
    case MCB_IN_ACC:
    case MCB_OUT_ACC:
    case MCB_DOCK_ACC:
        // currently not handled, though received
        break;
    default:
        log_error("Unknown MCB ack received");
        break;
    }
}

bool StratoPIB::StartMCBMotion()
{
    bool success = false;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        snprintf(log_array, LOG_ARRAY_SIZE, "Retracting %0.1f revs", retract_length);
        success = mcbComm.TX_Reel_In(retract_length, pib_config.retract_velocity); // todo: verification
        break;
    case MOTION_REEL_OUT:
        snprintf(log_array, LOG_ARRAY_SIZE, "Deploying %0.1f revs", deploy_length);
        success = mcbComm.TX_Reel_Out(deploy_length, pib_config.deploy_velocity); // todo: verification
        break;
    case MOTION_DOCK:
        snprintf(log_array, LOG_ARRAY_SIZE, "Docking %0.1f revs", dock_length);
        success = mcbComm.TX_Dock(dock_length, pib_config.dock_velocity); // todo: verification
        break;
    case MOTION_UNDOCK:
    default:
        mcb_motion = NO_MOTION;
        log_error("Unknown motion type to start");
        return false;
    }

    if (autonomous_mode) {
        log_nominal(log_array);
    } else {
        ZephyrLogFine(log_array);
    }

    return success;
}

bool StratoPIB::ScheduleProfiles()
{
    // no matter the trigger, reset the time_trigger to the max value, new TC needed to set new value
    if (!EEPROM_UPDATE_UINT32(pibStorage, time_trigger, UINT32_MAX)) {
        // should never happen, probably should reset
        return false;
    }

    // schedule the configured number of profiles starting in five seconds
    for (int i = 0; i < pib_config.num_profiles; i++) {
        if (!scheduler.AddAction(COMMAND_BEGIN_PROFILE, i * pib_config.profile_period + 5)) return false;
    }

    profiles_remaining = pib_config.num_profiles;

    return true;
}