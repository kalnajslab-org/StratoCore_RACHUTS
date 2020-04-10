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
    , puComm(&PU_SERIAL)
{
}

// --------------------------------------------------------
// General instrument functions
// --------------------------------------------------------

// note serial setup occurs in main arduino file
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

    // PU power switch
    pinMode(PU_PWR_ENABLE, OUTPUT);
    digitalWrite(PU_PWR_ENABLE, LOW);

    if (!pibConfigs.Initialize()) {
        ZephyrLogWarn("Error loading from EEPROM! Reconfigured");
    }

    mcbComm.AssignBinaryRXBuffer(binary_mcb, MCB_BUFFER_SIZE);
    puComm.AssignBinaryRXBuffer(binary_pu, PU_BUFFER_SIZE);
}

void StratoPIB::InstrumentLoop()
{
    WatchFlags();
}

// --------------------------------------------------------
// Action handler and action flag helper functions
// --------------------------------------------------------

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

// --------------------------------------------------------
// Profile helpers
// --------------------------------------------------------

bool StratoPIB::StartMCBMotion()
{
    bool success = false;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        snprintf(log_array, LOG_ARRAY_SIZE, "Retracting %0.1f revs", retract_length);
        success = mcbComm.TX_Reel_In(retract_length, pibConfigs.retract_velocity.Read());
        max_profile_seconds = 60 * (retract_length / pibConfigs.retract_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
    case MOTION_REEL_OUT:
        PUUndock();
        snprintf(log_array, LOG_ARRAY_SIZE, "Deploying %0.1f revs", deploy_length);
        success = mcbComm.TX_Reel_Out(deploy_length, pibConfigs.deploy_velocity.Read());
        max_profile_seconds = 60 * (deploy_length / pibConfigs.deploy_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
    case MOTION_DOCK:
        snprintf(log_array, LOG_ARRAY_SIZE, "Docking %0.1f revs", dock_length);
        success = mcbComm.TX_Dock(dock_length, pibConfigs.dock_velocity.Read());
        max_profile_seconds = 60 * (dock_length / pibConfigs.dock_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
    case MOTION_IN_NO_LW:
        snprintf(log_array, LOG_ARRAY_SIZE, "Reel in (no LW) %0.1f revs", retract_length);
        success = mcbComm.TX_In_No_LW(retract_length, pibConfigs.dock_velocity.Read());
        max_profile_seconds = 60 * (retract_length / pibConfigs.dock_velocity.Read()) + pibConfigs.motion_timeout.Read();
        break;
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
    pibConfigs.time_trigger.Write(UINT32_MAX);

    // schedule the configured number of profiles starting in five seconds
    for (int i = 0; i < pibConfigs.num_profiles.Read(); i++) {
        if (!scheduler.AddAction(ACTION_BEGIN_PROFILE, i * pibConfigs.profile_period.Read() + 5)) {
            ZephyrLogCrit("Error scheduling profiles, scheduler failure");
            return false;
        }
    }

    snprintf(log_array, LOG_ARRAY_SIZE, "Scheduled profiles: %u, %0.2f, %0.2f, %0.2f, %u, %u", pibConfigs.num_profiles.Read(),
             pibConfigs.profile_size.Read(), pibConfigs.dock_amount.Read(), pibConfigs.dock_overshoot.Read(),
             pibConfigs.dwell_time.Read(), pibConfigs.profile_period.Read());
    ZephyrLogFine(log_array);
    return true;
}

void StratoPIB::AddMCBTM()
{
    // make sure it's the correct size
    if (mcbComm.binary_rx.bin_length != MOTION_TM_SIZE) {
        log_error("invalid motion TM size");
        return;
    }

    // sync byte
    if (!zephyrTX.addTm((uint8_t) 0xA5)) {
        log_error("unable to add sync byte to MCB TM buffer");
        return;
    }

    // tenths of seconds since start
    if (!zephyrTX.addTm((uint16_t) ((millis() - profile_start) / 100))) {
        log_error("unable to add seconds bytes to MCB TM buffer");
        return;
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        if (!zephyrTX.addTm(mcbComm.binary_rx.bin_buffer[i])) {
            log_error("unable to add data byte to MCB TM buffer");
            return;
        }
    }
}

void StratoPIB::NoteProfileStart()
{
    mcb_motion_ongoing = true;
    profile_start = millis();

    if (MOTION_DOCK == mcb_motion || MOTION_IN_NO_LW == mcb_motion) mcb_dock_ongoing = true;

    zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data

    // MCB TM Header
    zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
    // add to header: profile type, auto vs. manual, auto trigger?
}

void StratoPIB::SendMCBTM(StateFlag_t state_flag, const char * message)
{
    // use only the first flag to report the motion
    zephyrTX.setStateDetails(1, message);
    zephyrTX.setStateFlagValue(1, state_flag);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal(log_array);

    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
}

void StratoPIB::SendTSENTM()
{
    if (0 < snprintf(log_array, LOG_ARRAY_SIZE, "PU TSEN: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", pu_status.time, pu_status.v_battery, pu_status.i_charge, pu_status.therm1, pu_status.therm2, pu_status.heater_stat)) {
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
    } else {
        zephyrTX.setStateDetails(1, "PU TSEN: unable to add status info");
        zephyrTX.setStateFlagValue(1, WARN);
    }

    // use only the first flag to report the motion
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal(log_array);
}

void StratoPIB::SendProfileTM(uint8_t packet_num)
{
    if (0 < snprintf(log_array, LOG_ARRAY_SIZE, "PU Profile Record %u: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", packet_num, pu_status.time, pu_status.v_battery, pu_status.i_charge, pu_status.therm1, pu_status.therm2, pu_status.heater_stat)) {
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
    } else {
        zephyrTX.setStateDetails(1, "PU Profile Record: unable to add status info");
        zephyrTX.setStateFlagValue(1, WARN);
    }

    // use only the first flag to report the motion
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    zephyrTX.TM();

    log_nominal(log_array);
}

// every 15 minutes, synchronized with the hour
bool StratoPIB::ScheduleNextTSEN()
{
    int32_t temp_seconds = 0;
    int32_t delta_seconds = 0;
    TimeElements temp_exact;

    // get the current time in seconds and the TimeElements struct
    temp_seconds = now();
    breakTime(temp_seconds, temp_exact);

    // find the number of seconds until the next 15 minute time
    delta_seconds = (15 - (temp_exact.Minute % 15)) * 60;
    delta_seconds -= temp_exact.Second;

    // add the number of seconds to the current time
    temp_seconds += delta_seconds;

    // remake the struct for the exact desired scheduled time
    breakTime(temp_seconds, temp_exact);

    return scheduler.AddAction(COMMAND_SEND_TSEN, temp_exact);
}

void StratoPIB::PUDock()
{
    pibConfigs.pu_docked.Write(true);
    digitalWrite(PU_PWR_ENABLE, HIGH);
}

void StratoPIB::PUUndock()
{
    pibConfigs.pu_docked.Write(false);
    digitalWrite(PU_PWR_ENABLE, LOW);
}

void StratoPIB::PUStartProfile()
{
    int32_t t_down = 60 * (deploy_length / pibConfigs.deploy_velocity.Read()) + pibConfigs.preprofile_time.Read();
    int32_t t_up = 60 * (retract_length / pibConfigs.retract_velocity.Read() + dock_length / pibConfigs.dock_velocity.Read())
                   + pibConfigs.motion_timeout.Read(); // extra time for dock delay

    puComm.TX_Profile(t_down, pibConfigs.dwell_time.Read(), t_up, pibConfigs.profile_rate.Read(), pibConfigs.dwell_rate.Read(),
                      pibConfigs.profile_TSEN.Read(), pibConfigs.profile_ROPC.Read(), pibConfigs.profile_FLASH.Read());
}