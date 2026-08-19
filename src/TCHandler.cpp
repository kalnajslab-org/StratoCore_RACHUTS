/*
 *  TCHandler.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Telecommand handler.
 */

#include "StratoRachuts.h"

// Guard for flight-only TCs. mode_code is set at the top of each mode function,
// so it reflects the current StratoCore mode when a TC is handled. On failure it
// populates the TC-ack detail (msg3) and flag rather than logging directly.
bool StratoRachuts::RequireFlightMode(const char * cmd, String & msg3, StateFlag_t & flag)
{
    if (0 != strcmp(mode_code, "FL")) {
        msg3 = String(cmd) + " ignored: not in flight mode";
        flag = WARN;
        return false;
    }
    return true;
}

// The telecommand handler must return ACK/NAK
bool StratoRachuts::TCHandler(Telecommand_t telecommand)
{
    // TC acknowledgement summary (sent as a RACHUTSTCACK TM after the switch):
    // msg2 = command summary, msg3 = detail/error, msg1_flag = FINE/WARN/CRIT.
    String msg2("");
    String msg3("");
    StateFlag_t msg1_flag = FINE;

    // Deferred actions that send their own TM (run after the ack TM).
    bool send_pib_eeprom = false;

    switch (telecommand) {

    // MCB Telecommands -----------------------------------
    case DEPLOYx:
        msg2 = "TC Deploy Length";
        deploy_length = mcbParam.deployLen;
        msg2 += ": " + String(deploy_length, 1) + " revs";
        SetAction(ACTION_REEL_OUT); // will be ignored if wrong mode
        break;
    case DEPLOYv:
        pibConfigs.deploy_velocity.Write(mcbParam.deployVel);
        msg2 = "Set deploy_velocity: " + String(pibConfigs.deploy_velocity.Read(), 2);
        break;
    case DEPLOYa:
        msg2 = "TC Deploy Acceleration: " + String(mcbParam.deployAcc, 2);
        if (!mcbComm.TX_Out_Acc(mcbParam.deployAcc)) {
            msg3 = "Error sending deploy acc to MCB";
            msg1_flag = WARN;
        }
        break;
    case RETRACTx:
        msg2 = "TC Retract Length";
        retract_length = mcbParam.retractLen;
        msg2 += ": " + String(retract_length, 1) + " revs";
        SetAction(ACTION_REEL_IN); // will be ignored if wrong mode
        break;
    case RETRACTv:
        pibConfigs.retract_velocity.Write(mcbParam.retractVel);
        msg2 = "Set retract_velocity: " + String(pibConfigs.retract_velocity.Read(), 2);
        break;
    case RETRACTa:
        msg2 = "TC Retract Acceleration: " + String(mcbParam.retractAcc, 2);
        if (!mcbComm.TX_In_Acc(mcbParam.retractAcc)) {
            msg3 = "Error sending retract acc to MCB";
            msg1_flag = WARN;
        }
        break;
    case DOCKx:
        msg2 = "TC Dock Length";
        dock_length = mcbParam.dockLen;
        msg2 += ": " + String(dock_length, 1) + " revs";
        SetAction(ACTION_DOCK); // will be ignored if wrong mode
        break;
    case DOCKv:
        pibConfigs.dock_velocity.Write(mcbParam.dockVel);
        msg2 = "Set dock_velocity: " + String(pibConfigs.dock_velocity.Read(), 2);
        break;
    case DOCKa:
        msg2 = "TC Dock Acceleration: " + String(mcbParam.dockAcc, 2);
        if (!mcbComm.TX_Dock_Acc(mcbParam.dockAcc)) {
            msg3 = "Error sending dock acc to MCB";
            msg1_flag = WARN;
        }
        break;
    case FULLRETRACT:
        // todo: determine implementation
        msg2 = "TC Full Retract";
        msg3 = "TC Full Retract not implemented";
        msg1_flag = WARN;
        break;
    case CANCELMOTION:
        msg2 = "TC Cancel Motion";
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(ACTION_MOTION_STOP);
        break;
    case ZEROREEL:
        msg2 = "TC Zero Reel";
        if (mcb_dock_ongoing) {
            msg3 = "Can't zero reel, motion ongoing";
            msg1_flag = WARN;
        } else {
            mcbComm.TX_ASCII(MCB_ZERO_REEL);
        }
        break;
    case CENTERLW:
        msg2 = "TC Center Level Wind";
        if (mcb_motion_ongoing) {
            msg3 = "Can't center level wind, motion ongoing";
            msg1_flag = WARN;
        } else {
            mcbComm.TX_ASCII(MCB_CENTER_LW);
        }
        break;
    case TEMPLIMITS:
        msg2 = "TC Set Temperature Limits";
        if (!mcbComm.TX_Temp_Limits(mcbParam.tempLimits[0],mcbParam.tempLimits[1],mcbParam.tempLimits[2],mcbParam.tempLimits[3],mcbParam.tempLimits[4],mcbParam.tempLimits[5])) {
            msg3 = "Error sending temperature limits to MCB";
            msg1_flag = WARN;
        }
        break;
    case TORQUELIMITS:
        msg2 = "TC Set Torque Limits";
        if (!mcbComm.TX_Torque_Limits(mcbParam.torqueLimits[0],mcbParam.torqueLimits[1])) {
            msg3 = "Error sending torque limits to MCB";
            msg1_flag = WARN;
        }
        break;
    case CURRLIMITS:
        msg2 = "TC Set Current Limits";
        if (!mcbComm.TX_Curr_Limits(mcbParam.currLimits[0],mcbParam.currLimits[1])) {
            msg3 = "Error sending curr limits to MCB";
            msg1_flag = WARN;
        }
        break;
    case IGNORELIMITS:
        msg2 = "TC Ignore Limits";
        mcbComm.TX_ASCII(MCB_IGNORE_LIMITS);
        break;
    case USELIMITS:
        msg2 = "TC Use Limits";
        mcbComm.TX_ASCII(MCB_USE_LIMITS);
        break;
    case GETMCBEEPROM:
        msg2 = "TC Get MCB EEPROM";
        if (mcb_motion_ongoing) {
            msg3 = "Motion ongoing, request MCB EEPROM later";
            msg1_flag = WARN;
        } else {
            mcbComm.TX_ASCII(MCB_GET_EEPROM);
        }
        break;

    // PIB Telecommands -----------------------------------
    case SETPROFILESIZE:
        pibConfigs.profile_size.Write(pibParam.profileSize);
        msg2 = "Set profile_size: " + String(pibConfigs.profile_size.Read(), 2);
        break;
    case SETDOCKAMOUNT:
        pibConfigs.dock_amount.Write(pibParam.dockAmount);
        msg2 = "Set dock_amount: " + String(pibConfigs.dock_amount.Read(), 2);
        break;
    case SETDWELLTIME:
        pibConfigs.dwell_time.Write(pibParam.dwellTime);
        msg2 = "Set dwell_time: " + String(pibConfigs.dwell_time.Read());
        break;
    case SETDOCKOVERSHOOT:
        pibConfigs.dock_overshoot.Write(pibParam.dockOvershoot);
        msg2 = "Set dock_overshoot: " + String(pibConfigs.dock_overshoot.Read(), 2);
        break;
    case RETRYDOCK:
        msg2 = "TC Retry Dock";
        if (!RequireFlightMode("Retry dock", msg3, msg1_flag)) break;
        deploy_length = mcbParam.deployLen;
        retract_length = mcbParam.retractLen;
        msg2 += ": deploy=" + String(deploy_length, 1) + " revs, retract=" + String(retract_length, 1) + " revs";
        SetAction(COMMAND_REDOCK);
        break;
    case GETPUSTATUS:
        msg2 = "TC Get PU Status";
        if (!RequireFlightMode("Get PU status", msg3, msg1_flag)) break;
        SetAction(ACTION_CHECK_PU);
        break;
    case PUPOWERON:
        msg2 = "PU powered on";
        digitalWrite(PU_PWR_ENABLE, HIGH);
        break;
    case PUPOWEROFF:
        msg2 = "PU powered off";
        digitalWrite(PU_PWR_ENABLE, LOW);
        break;
    case MANUALPROFILE:
        msg2 = "TC Manual Profile";
        if (!RequireFlightMode("Manual profile", msg3, msg1_flag)) break;
        pibConfigs.profile_size.Write(pibParam.profileSize);
        pibConfigs.dock_amount.Write(pibParam.dockAmount);
        pibConfigs.dock_overshoot.Write(pibParam.dockOvershoot);
        pibConfigs.dwell_time.Write(pibParam.dwellTime);
        msg2 += ": size=" + String(pibParam.profileSize, 1) + " revs, dock=" + String(pibParam.dockAmount, 1)
              + " revs, overshoot=" + String(pibParam.dockOvershoot, 1) + " revs, dwell=" + String(pibParam.dwellTime) + "s";
        SetAction(COMMAND_MANUAL_PROFILE);
        break;
    case OFFLOADPUPROFILE:
        msg2 = "TC Offload PU Profile";
        if (!RequireFlightMode("PU profile offload", msg3, msg1_flag)) break;
        SetAction(ACTION_OFFLOAD_PU);
        break;
    case SETPREPROFILETIME:
        pibConfigs.preprofile_time.Write(pibParam.preprofileTime);
        msg2 = "Set preprofile_time: " + String(pibConfigs.preprofile_time.Read());
        break;
    case SETPUWARMUPTIME:
        pibConfigs.puwarmup_time.Write(pibParam.warmupTime);
        msg2 = "Set puwarmup_time: " + String(pibConfigs.puwarmup_time.Read());
        break;
    case AUTOREDOCKPARAMS:
        pibConfigs.redock_out.Write(pibParam.autoRedockOut);
        pibConfigs.redock_in.Write(pibParam.autoRedockIn);
        pibConfigs.num_redock.Write(pibParam.numRedock);
        msg2 = "New auto redock params: " + String(pibConfigs.redock_out.Read(), 2) + ", "
             + String(pibConfigs.redock_in.Read(), 2) + ", " + String(pibConfigs.num_redock.Read());
        break;
    case SETMOTIONTIMEOUT:
        pibConfigs.motion_timeout.Write(pibParam.motionTimeout);
        msg2 = "Set motion_timeout: " + String(pibConfigs.motion_timeout.Read());
        break;
    case GETPIBEEPROM:
        msg2 = "TC Get RACHuTS EEPROM";
        if (mcb_motion_ongoing) {
            msg3 = "Motion ongoing, request RACHuTS EEPROM later";
            msg1_flag = WARN;
        } else {
            send_pib_eeprom = true;
        }
        break;
    case DOCKEDPROFILE:
        msg2 = "TC Docked Profile";
        if (!RequireFlightMode("Docked profile", msg3, msg1_flag)) break;
        docked_profile_time = pibParam.dockedProfileTime;
        docked_profile_rate = pibParam.dockedProfileRate;
        msg2 += ": length=" + String(docked_profile_time) + "s"
              + " rate=" + String(docked_profile_rate) + "s"
              + " offload=" + String(pibConfigs.docked_offload_period.Read()) + "s"
              + " ROPC=" + String(pibConfigs.rpu_enable_ROPC.Read())
              + " TDLAS=" + String(pibConfigs.rpu_enable_TDLAS.Read())
              + " TSEN=" + String(pibConfigs.rpu_enable_TSEN.Read())
              + " RS41=" + String(pibConfigs.rpu_enable_RS41.Read());
        SetAction(COMMAND_DOCKED_PROFILE);
        break;
    case STARTREALTIMEMCB:
        msg2 = "TC Start Real-Time MCB";
        if (mcb_motion_ongoing) {
            msg3 = "Cannot start real-time MCB mode, motion ongoing";
            msg1_flag = WARN;
        } else {
            pibConfigs.real_time_mcb.Write(true);
        }
        break;
    case EXITREALTIMEMCB:
        msg2 = "TC Exit Real-Time MCB";
        if (mcb_motion_ongoing) {
            msg3 = "Cannot exit real-time MCB mode, motion ongoing";
            msg1_flag = WARN;
        } else {
            pibConfigs.real_time_mcb.Write(false);
        }
        break;
    case CANCELMEASURE:
        msg2 = "TC Cancel Measure";
        puComm.TX_GoStandby(pibConfigs.rpu_bat_temp.Read()); // no matter what, attempt to send (irrespective of mode)
        SetAction(ACTION_CANCEL_MEASURE);
        break;
    case SETDOCKEDOFFLOADPERIOD:
        msg2 = "TC Set Docked Offload Period";
        // Reject rather than just warn, below the TM resend timeout (not at it --
        // a period equal to the timeout is a natural choice for bench testing and
        // is not itself a problem): a shorter period starts the next offload
        // before the previous offload's RESEND_TM entries have aged out of the
        // scheduler queue (KnownIssues 3). Leaves the previously stored value in
        // place.
        if (0 < pibParam.dockedOffloadPeriod && pibParam.dockedOffloadPeriod < ZEPHYR_RESEND_TIMEOUT) {
            msg3 = "Rejected: period < " + String(ZEPHYR_RESEND_TIMEOUT) + "s min";
            msg1_flag = WARN;
        } else {
            pibConfigs.docked_offload_period.Write(pibParam.dockedOffloadPeriod);
            msg2 += ": " + String(pibConfigs.docked_offload_period.Read());
        }
        break;

    // PU Telecommands ------------------------------------
    case RPUBATTEMP:
        pibConfigs.rpu_bat_temp.Write(rpuParam.batTemp);
        msg2 = "Set rpu_bat_temp: " + String(pibConfigs.rpu_bat_temp.Read(), 2);
        break;
    case RPURESET:
        msg2 = "TC RPU Reset";
        puComm.TX_ASCII(RPU_RESET);
        break;
    case RPUCONFIG:
        pibConfigs.rpu_meas_duration.Write(rpuParam.measDurationSecs);
        pibConfigs.rpu_meas_rate.Write(rpuParam.measRateSecs);
        pibConfigs.rpu_enable_ROPC.Write(rpuParam.enableROPC);
        pibConfigs.rpu_enable_TDLAS.Write(rpuParam.enableTDLAS);
        pibConfigs.rpu_enable_TSEN.Write(rpuParam.enableTSEN);
        pibConfigs.rpu_enable_RS41.Write(rpuParam.enableRS41);
        msg2 = "RPU config: duration=" + String(pibConfigs.rpu_meas_duration.Read())
             + " rate=" + String(pibConfigs.rpu_meas_rate.Read())
             + " ROPC=" + String(pibConfigs.rpu_enable_ROPC.Read())
             + " TDLAS=" + String(pibConfigs.rpu_enable_TDLAS.Read())
             + " TSEN=" + String(pibConfigs.rpu_enable_TSEN.Read())
             + " RS41=" + String(pibConfigs.rpu_enable_RS41.Read());
        break;
    case RPUSTATUSPERIOD:
        pibConfigs.rpu_status_rate.Write(rpuParam.statusPeriodSecs);
        puComm.TX_SetStatusRate(pibConfigs.rpu_status_rate.Read());
        msg2 = "Set rpu_status_rate: " + String(pibConfigs.rpu_status_rate.Read());
        break;
    case RPUGOSTANDBY:
        msg2 = "Sent go-standby to RPU";
        puComm.TX_GoStandby(pibConfigs.rpu_bat_temp.Read());
        break;
    case RPUGOMEASURE:
        // Duration and rate come from the TC parameters; battery setpoint and
        // sensor-enable flags still come from the stored RPU config.
        puComm.TX_GoMeasure(rpuParam.measDurationSecs,
                            rpuParam.measRateSecs,
                            pibConfigs.rpu_bat_temp.Read(),
                            pibConfigs.rpu_enable_ROPC.Read(), pibConfigs.rpu_enable_TDLAS.Read(),
                            pibConfigs.rpu_enable_TSEN.Read(), pibConfigs.rpu_enable_RS41.Read());
        msg2 = "Sent go-measure to RPU: duration=" + String(rpuParam.measDurationSecs)
             + " rate=" + String(rpuParam.measRateSecs);
        break;

    // General Telecommands -------------------------------
    // note that RESET_INST and GETTMBUFFER are implemented in StratoCore
    case EXITERROR:
        msg2 = "TC Exit Error";
        SetAction(EXIT_ERROR_STATE);
        break;

    // Error case -----------------------------------------
    default:
        msg1_flag = CRIT;
        msg3 = "Unknown TC " + String(telecommand) + " received";
        break;
    }

    // Send a TC acknowledgement TM
    zephyrTX.clearTm();
    zephyrTX.setStateDetails(1, "RACHUTSTCACK");
    zephyrTX.setStateFlagValue(1, msg1_flag);

    zephyrTX.setStateDetails(2, msg2);
    zephyrTX.setStateFlagValue(2, FINE);

    zephyrTX.setStateDetails(3, msg3);
    zephyrTX.setStateFlagValue(3, FINE);

    TM_ack_flag = NO_ACK;
    ZephyrTXpoke(ZEPHYRTX_TM);

    // Log the TC summary message
    switch (msg1_flag) {
    case FINE:
        log_nominal(msg2.c_str());
        break;
    case WARN:
    case CRIT:
        log_error(msg2.c_str());
        break;
    default:
        log_debug(msg2.c_str());
    }

    // Deferred actions that send their own TM (run after the ack TM)
    if (send_pib_eeprom) {
        SendPIBEEPROM();
    }

    return true;
}
