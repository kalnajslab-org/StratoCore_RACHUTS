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

    if (!pibStorage.LoadFromEEPROM()) {
        ZephyrLogWarn("EEPROM updated");
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
    case MOTION_IN_NO_LW:
        snprintf(log_array, LOG_ARRAY_SIZE, "Reel in (no LW) %0.1f revs", retract_length);
        success = mcbComm.TX_In_No_LW(retract_length, pib_config.dock_velocity); // todo: verification
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
    if (!EEPROM_UPDATE_UINT32(pibStorage, time_trigger, UINT32_MAX)) {
        // should never happen, probably should reset
        return false;
    }

    // schedule the configured number of profiles starting in five seconds
    for (int i = 0; i < pib_config.num_profiles; i++) {
        if (!scheduler.AddAction(ACTION_BEGIN_PROFILE, i * pib_config.profile_period + 5)) return false;
    }

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

    if (MOTION_REEL_OUT == mcb_motion) EEPROM_UPDATE_BOOL(pibStorage, pu_docked, false);

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

    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
}

void StratoPIB::SendTSENTM()
{
    char message[100] = "";

    if (0 < snprintf(message, 100, "PU TSEN: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", PUTime, PUVBattery, PUICharge, PUTherm1T, PUTherm2T, PUHeaterStat)) {
        zephyrTX.setStateDetails(1, message);
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