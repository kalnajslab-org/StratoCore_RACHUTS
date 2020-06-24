/*
 *  TCHandler.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Telecommand handler.
 */

#include "StratoPIB.h"

// The telecommand handler must return ACK/NAK
void StratoPIB::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";
    log_debug("Received telecommand");

    switch (telecommand) {

    // MCB Telecommands -----------------------------------
    case DEPLOYx:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        deploy_length = mcbParam.deployLen;
        SetAction(ACTION_REEL_OUT); // will be ignored if wrong mode
        SetAction(ACTION_OVERRIDE_TSEN);
        break;
    case DEPLOYv:
        pibConfigs.deploy_velocity.Write(mcbParam.deployVel);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set deploy_velocity: %f", pibConfigs.deploy_velocity.Read());
        ZephyrLogFine(log_array);
        break;
    case DEPLOYa:
        if (!mcbComm.TX_Out_Acc(mcbParam.deployAcc)) {
            ZephyrLogWarn("Error sending deploy acc to MCB");
        }
        break;
    case RETRACTx:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        retract_length = mcbParam.retractLen;
        SetAction(ACTION_REEL_IN); // will be ignored if wrong mode
        SetAction(ACTION_OVERRIDE_TSEN);
        break;
    case RETRACTv:
        pibConfigs.retract_velocity.Write(mcbParam.retractVel);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set retract_velocity: %f", pibConfigs.retract_velocity.Read());
        ZephyrLogFine(log_array);
        break;
    case RETRACTa:
        if (!mcbComm.TX_In_Acc(mcbParam.retractAcc)) {
            ZephyrLogWarn("Error sending retract acc to MCB");
        }
        break;
    case DOCKx:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        dock_length = mcbParam.dockLen;
        SetAction(ACTION_DOCK); // will be ignored if wrong mode
        SetAction(ACTION_OVERRIDE_TSEN);
        break;
    case DOCKv:
        pibConfigs.dock_velocity.Write(mcbParam.dockVel);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set dock_velocity: %f", pibConfigs.dock_velocity.Read());
        ZephyrLogFine(log_array);
        break;
    case DOCKa:
        if (!mcbComm.TX_Dock_Acc(mcbParam.dockAcc)) {
            ZephyrLogWarn("Error sending dock acc to MCB");
        }
        break;
    case FULLRETRACT:
        // todo: determine implementation
        break;
    case CANCELMOTION:
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(ACTION_MOTION_STOP);
        SetAction(ACTION_OVERRIDE_TSEN);
        break;
    case ZEROREEL:
        if (mcb_dock_ongoing) {
            ZephyrLogWarn("Can't zero reel, motion ongoing");
        }

        mcbComm.TX_ASCII(MCB_ZERO_REEL);
        break;
    case TEMPLIMITS:
        if (!mcbComm.TX_Temp_Limits(mcbParam.tempLimits[0],mcbParam.tempLimits[1],mcbParam.tempLimits[2],mcbParam.tempLimits[3],mcbParam.tempLimits[4],mcbParam.tempLimits[5])) {
            ZephyrLogWarn("Error sending temperature limits to MCB");
        }
        break;
    case TORQUELIMITS:
        if (!mcbComm.TX_Torque_Limits(mcbParam.torqueLimits[0],mcbParam.torqueLimits[1])) {
            ZephyrLogWarn("Error sending torque limits to MCB");
        }
        break;
    case CURRLIMITS:
        if (!mcbComm.TX_Curr_Limits(mcbParam.currLimits[0],mcbParam.currLimits[1])) {
            ZephyrLogWarn("Error sending curr limits to MCB");
        }
        break;
    case IGNORELIMITS:
        mcbComm.TX_ASCII(MCB_IGNORE_LIMITS);
        break;
    case USELIMITS:
        mcbComm.TX_ASCII(MCB_USE_LIMITS);
        break;
    case GETMCBEEPROM:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion ongoing, request MCB EEPROM later");
        } else {
            mcbComm.TX_ASCII(MCB_GET_EEPROM);
        }
        break;

    // PIB Telecommands -----------------------------------
    case SETAUTO:
        if (!mcb_motion_ongoing) {
            autonomous_mode = true;
            inst_substate = MODE_ENTRY; // restart FL in auto
            ZephyrLogFine("Set mode to auto");
        } else {
            ZephyrLogWarn("Motion ongoing, can't update mode");
        }
        break;
    case SETMANUAL:
        if (!mcb_motion_ongoing) {
            autonomous_mode = false;
            inst_substate = MODE_ENTRY; // restart FL in manual
            ZephyrLogFine("Set mode to manual");
        } else {
            ZephyrLogWarn("Motion ongoing, can't update mode");
        }
        break;
    case SETSZAMIN:
        pibConfigs.sza_minimum.Write(pibParam.szaMinimum);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set sza_minimum: %f", pibConfigs.sza_minimum.Read());
        ZephyrLogFine(log_array);
        break;
    case SETPROFILESIZE:
        pibConfigs.profile_size.Write(pibParam.profileSize);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set profile_size: %f", pibConfigs.profile_size.Read());
        ZephyrLogFine(log_array);
        break;
    case SETDOCKAMOUNT:
        pibConfigs.dock_amount.Write(pibParam.dockAmount);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set dock_amount: %f", pibConfigs.dock_amount.Read());
        ZephyrLogFine(log_array);
        break;
    case SETDWELLTIME:
        pibConfigs.dwell_time.Write(pibParam.dwellTime);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set dwell_time: %u", pibConfigs.dwell_time.Read());
        ZephyrLogFine(log_array);
        break;
    case SETPROFILEPERIOD:
        pibConfigs.profile_period.Write(pibParam.profilePeriod);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set profile_period: %u", pibConfigs.profile_period.Read());
        ZephyrLogFine(log_array);
        break;
    case SETNUMPROFILES:
        pibConfigs.num_profiles.Write(pibParam.numProfiles);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set num_profiles: %u", pibConfigs.num_profiles.Read());
        ZephyrLogFine(log_array);
        break;
    case SETTIMETRIGGER:
        if ((uint32_t) now() > pibParam.timeTrigger) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Can't use time trigger in past: %lu is less than %lu", pibParam.timeTrigger, (uint32_t) now());
            ZephyrLogWarn(log_array);
            break;
        }
        pibConfigs.time_trigger.Write(pibParam.timeTrigger);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set time_trigger: %lu", pibConfigs.time_trigger.Read());
        ZephyrLogFine(log_array);
        profiles_remaining = pibConfigs.num_profiles.Read();
        break;
    case USESZATRIGGER:
        pibConfigs.sza_trigger.Write(true);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set sza_trigger: %u", pibConfigs.sza_trigger.Read());
        ZephyrLogFine(log_array);
        break;
    case USETIMETRIGGER:
        pibConfigs.sza_trigger.Write(false);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set sza_trigger: %u", pibConfigs.sza_trigger.Read());
        ZephyrLogFine(log_array);
        break;
    case SETDOCKOVERSHOOT:
        pibConfigs.dock_overshoot.Write(pibParam.dockOvershoot);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set dock_overshoot: %f", pibConfigs.dock_overshoot.Read());
        ZephyrLogFine(log_array);
        break;
    case RETRYDOCK:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        log_nominal("Received retry dock telecommand");

        // schedule each action
        SetAction(COMMAND_REDOCK);
        SetAction(ACTION_OVERRIDE_TSEN);

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
        ZephyrLogFine("PU powered on");
        break;
    case PUPOWEROFF:
        digitalWrite(PU_PWR_ENABLE, LOW);
        ZephyrLogFine("PU powered off");
        break;
    case MANUALPROFILE:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding motion");
            break;
        }
        log_nominal("Received manual profile telecommand");

        pibConfigs.profile_size.Write(pibParam.profileSize);
        pibConfigs.dock_amount.Write(pibParam.dockAmount);
        pibConfigs.dock_overshoot.Write(pibParam.dockOvershoot);
        pibConfigs.dwell_time.Write(pibParam.dwellTime);

        // schedule each action
        SetAction(COMMAND_MANUAL_PROFILE);
        SetAction(ACTION_OVERRIDE_TSEN);
        break;
    case OFFLOADPUPROFILE:
        if (autonomous_mode) {
            ZephyrLogWarn("PU Profile offload TC only implemented for manual");
            break;
        }

        log_nominal("Received offload PU profile TC");

        SetAction(ACTION_OFFLOAD_PU);
        SetAction(ACTION_OVERRIDE_TSEN);
        break;
    case SETPREPROFILETIME:
        pibConfigs.preprofile_time.Write(pibParam.preprofileTime);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set preprofile_time: %u", pibConfigs.preprofile_time.Read());
        ZephyrLogFine(log_array);
        break;
    case SETPUWARMUPTIME:
        pibConfigs.puwarmup_time.Write(pibParam.warmupTime);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set puwarmup_time: %u", pibConfigs.puwarmup_time.Read());
        ZephyrLogFine(log_array);
        break;
    case AUTOREDOCKPARAMS:
        pibConfigs.redock_out.Write(pibParam.autoRedockOut);
        pibConfigs.redock_in.Write(pibParam.autoRedockIn);
        pibConfigs.num_redock.Write(pibParam.numRedock);
        snprintf(log_array, LOG_ARRAY_SIZE, "New auto redock params: %0.2f, %0.2f, %u", pibConfigs.redock_out.Read(),
                 pibConfigs.redock_in.Read(), pibConfigs.num_redock.Read());
        ZephyrLogFine(log_array);
        break;
    case SETMOTIONTIMEOUT:
        pibConfigs.motion_timeout.Write(pibParam.motionTimeout);
        snprintf(log_array, LOG_ARRAY_SIZE, "Set motion_timeout: %u", pibConfigs.motion_timeout.Read());
        ZephyrLogFine(log_array);
        break;
    case GETPIBEEPROM:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Motion ongoing, request PIB EEPROM later");
        } else {
            SendPIBEEPROM();
        }
        break;
    case DOCKEDPROFILE:
        if (autonomous_mode) {
            ZephyrLogWarn("Switch to manual mode before commanding docked profile");
            break;
        }
        log_nominal("Received docked profile telecommand");

        // set the duration
        docked_profile_time = pibParam.dockedProfileTime;

        // schedule each action
        SetAction(COMMAND_DOCKED_PROFILE);
        SetAction(ACTION_OVERRIDE_TSEN);
        break;
    case STARTREALTIMEMCB:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Cannot start real-time MCB mode, motion ongoing");
        } else {
            pibConfigs.real_time_mcb.Write(true);
            ZephyrLogFine("Started real-time MCB mode");
        }
        break;
    case EXITREALTIMEMCB:
        if (mcb_motion_ongoing) {
            ZephyrLogWarn("Cannot exit real-time MCB mode, motion ongoing");
        } else {
            pibConfigs.real_time_mcb.Write(false);
            ZephyrLogFine("Exited real-time MCB mode");
        }
        break;

    // PU Telecommands ------------------------------------
    case PUWARMUPCONFIGS:
        pibConfigs.flash_temp.Write(puParam.flashT);
        pibConfigs.heater1_temp.Write(puParam.heater1T);
        pibConfigs.heater2_temp.Write(puParam.heater2T);
        pibConfigs.flash_power.Write(puParam.flashPower);
        pibConfigs.tsen_power.Write(puParam.tsenPower);
        snprintf(log_array, LOG_ARRAY_SIZE, "New PU warmup configs: %0.2f, %0.2f, %0.2f, %u, %u", pibConfigs.flash_temp.Read(),
                 pibConfigs.heater1_temp.Read(), pibConfigs.heater2_temp.Read(), pibConfigs.flash_power.Read(), pibConfigs.tsen_power.Read());
        ZephyrLogFine(log_array);
        break;
    case PUPROFILECONFIGS:
        pibConfigs.profile_rate.Write(puParam.profileRate);
        pibConfigs.dwell_rate.Write(puParam.dwellRate);
        pibConfigs.profile_TSEN.Write(puParam.profileTSEN);
        pibConfigs.profile_ROPC.Write(puParam.profileROPC);
        pibConfigs.profile_FLASH.Write(puParam.profileFLASH);
        snprintf(log_array, LOG_ARRAY_SIZE, "New PU profile configs: %lu, %lu, %u, %u, %u", pibConfigs.profile_rate.Read(),
                 pibConfigs.dwell_rate.Read(), pibConfigs.profile_TSEN.Read(), pibConfigs.profile_ROPC.Read(), pibConfigs.profile_FLASH.Read());
        ZephyrLogFine(log_array);
        break;
    case PURESET:
        puComm.TX_ASCII(PU_RESET);
        break;
    case PUDOCKEDCONFIGS:
        pibConfigs.docked_rate.Write(puParam.dockedRate);
        pibConfigs.docked_TSEN.Write(puParam.dockedTSEN);
        pibConfigs.docked_ROPC.Write(puParam.dockedROPC);
        pibConfigs.docked_FLASH.Write(puParam.dockedFLASH);
        snprintf(log_array, LOG_ARRAY_SIZE, "New PU docked profile configs: %lu, %u, %u, %u", pibConfigs.docked_rate.Read(),
                 pibConfigs.docked_TSEN.Read(), pibConfigs.docked_ROPC.Read(), pibConfigs.docked_FLASH.Read());
        ZephyrLogFine(log_array);
        break;

    // General Telecommands -------------------------------
    // note that RESET_INST and GETTMBUFFER are implemented in StratoCore
    case EXITERROR:
        SetAction(EXIT_ERROR_STATE);
        ZephyrLogFine("Received exit error command");
        break;

    // Error case -----------------------------------------
    default:
        snprintf(log_array, LOG_ARRAY_SIZE, "Unknown TC ID: %u", telecommand);
        ZephyrLogWarn(log_array);
        break;
    }
}