/*
 *  TCHandler.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Telecommand handler.
 */

#include "StratoPIB.h"

// The telecommand handler must return ACK/NAK
bool StratoPIB::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";
    log_debug("Received telecommand");

    switch (telecommand) {
    case DEPLOYx:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        deploy_length = mcbParam.deployLen;
        SetAction(ACTION_REEL_OUT); // will be ignored if wrong mode
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
        SetAction(ACTION_REEL_IN); // will be ignored if wrong mode
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
        SetAction(ACTION_DOCK); // will be ignored if wrong mode
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
        SetAction(ACTION_MOTION_STOP);
        break;
    case ZEROREEL:
        mcbComm.TX_ASCII(MCB_ZERO_REEL); // todo: verification + ack
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
            profiles_remaining = pib_config.num_profiles;
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
    case RETRYDOCK:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        log_nominal("Received retry dock telecommand");

        // schedule each action
        SetAction(COMMAND_REDOCK);

        // set the parameters
        deploy_length = mcbParam.deployLen;
        retract_length = mcbParam.retractLen;
        break;
    case GETPUSTATUS:
        if (autonomous_mode) {
            ZephyrLogWarn("PU Status TC only implemented for manual");
            break;
        }

        log_nominal("Received get PU status TC");

        SetAction(ACTION_CHECK_PU);
        break;
    case PUPOWERON:
        digitalWrite(PU_PWR_ENABLE, HIGH);
        break;
    case PUPOWEROFF:
        digitalWrite(PU_PWR_ENABLE, LOW);
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