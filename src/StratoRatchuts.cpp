/*
 *  StratoRatchuts.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  Updated for MonDo board: November 2020 (LEK)
 *  Updated for T4.1 and Mondo Rev E 2024 (LEK)
 *
 *  This file implements an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the RACHuTS Profiler Interface Board, or PIB.
 */

int PacketSize = 0;

//ISR for LoRa reception, needs to be outside the class for some reason
void onReceive(int Size)
{
    PacketSize = Size;
}

#include "StratoRatchuts.h"

StratoRatchuts::StratoRatchuts()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT, &DEBUG_SERIAL)
    , mcbComm(&MCB_SERIAL)
    , puComm(&PU_SERIAL)
{
}

// --------------------------------------------------------
// General instrument functions
// --------------------------------------------------------

// note serial setup occurs in main arduino file
void StratoRatchuts::InstrumentSetup()
{

    // safe pin required by Zephyr
    pinMode(SAFE_PIN, OUTPUT);
    digitalWrite(SAFE_PIN, LOW);

    // PU power switch
    pinMode(PU_PWR_ENABLE, OUTPUT);
    digitalWrite(PU_PWR_ENABLE, LOW);

    //Teensy Analog setup
    analogReadResolution(12); //Set to 12 bits (0 - 4095)
    analogReadAveraging(32); //average 32 samples


    // Set up the second SPI Port for the LoRa Module
    SPI1.setSCK(LORA_SCK);
    SPI1.setMISO(LORA_MISO);
    SPI1.setMOSI(LORA_MOSI);

    LoRa.setSPI(SPI1);
    LoRa.setPins(SS_PIN, RESET_PIN,INTERUPT_PIN);
    
    LoRaInit();  //initialize the LoRa modem

    LoRa.onReceive(onReceive);
    LoRa.receive();

    if (!pibConfigs.Initialize()) {
        ZephyrLogWarn("Error loading from EEPROM! Reconfigured");
    }

    mcbComm.AssignBinaryRXBuffer(binary_mcb, MCB_BUFFER_SIZE);
    puComm.AssignBinaryRXBuffer(binary_pu, PU_BUFFER_SIZE);
}

void StratoRatchuts::InstrumentLoop()
{
    WatchFlags();
    LoRaRX();
}

void StratoRatchuts::LoRaInit()
{
   if (!LoRa.begin(FREQUENCY)){
       ZephyrLogWarn("Starting LoRa failed!");
       Serial.println("WARN: LoRa Initializtion Failed");
    }
    delay(1);
    LoRa.setSpreadingFactor(SF);
    delay(1);
    LoRa.setSignalBandwidth(BANDWIDTH);
    delay(1);
    LoRa.setTxPower(RF_POWER);
}

void StratoRatchuts::LoRaRX()
{
    if (PacketSize > 0) {
        PacketSize = 0;
        Serial.print("LoRa pkt RSSI:");
        Serial.println(LoRa.packetRssi());

        int BytesToRead = LoRa.available();
        for (int i = 0; i < BytesToRead; i++)
            LoRa_RX_buffer[i] = LoRa.read();

        RPUPacket rpu_packet;
        if (rpu_packet.decode((const uint8_t*)LoRa_RX_buffer, BytesToRead))
        {
            String json_str = rpu_packet.toJSON();
            for (size_t i = 0; i < json_str.length(); i++) {
                Serial.write(json_str[i]);
                if (json_str[i] == ',') Serial.println();
            }
            Serial.println();

            SendRPUSTATUS(json_str, "LORA");
        }
        else
        {
            Serial.println("Failed to decode RPUPacket");
        }
    }
    return;
}

// Send the decoded RPU status (as JSON) to the ground as an RPUSTATUS TM
void StratoRatchuts::SendRPUSTATUS(const String& json, const String& source)
{
    zephyrTX.clearTm();

    zephyrTX.setStateDetails(1, "RPUSTATUS");
    zephyrTX.setStateDetails(2, source);
    zephyrTX.setStateDetails(3, "");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    zephyrTX.addTm((const uint8_t*)json.c_str(), json.length());

    ZephyrTXpoke(ZEPHYRTX_TM);
    zephyrTX.clearTm();
}

// Wake the MAX3381 transceiver with a throwaway byte (absorbing the dropped
// first byte after its 30 s inactivity powerdown), then send the requested
// Zephyr message.
void StratoRatchuts::ZephyrTXpoke(ZephyrTXMsgType_t msg_type)
{
    ZEPHYR_SERIAL.write('\n');
    switch (msg_type) {
    case ZEPHYRTX_TM:
        zephyrTX.TM();
        break;
    case ZEPHYRTX_S:
        zephyrTX.S();
        break;
    case ZEPHYRTX_IMR:
        zephyrTX.IMR();
        break;
    case ZEPHYRTX_RA:
        zephyrTX.RA();
        break;
    }
}

// --------------------------------------------------------
// Action handler and action flag helper functions
// --------------------------------------------------------

void StratoRatchuts::ActionHandler(uint8_t action)
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

bool StratoRatchuts::CheckAction(uint8_t action)
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

void StratoRatchuts::SetAction(uint8_t action)
{
    action_flags[action].flag_value = true;
    action_flags[action].stale_count = 0;
}

void StratoRatchuts::WatchFlags()
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

bool StratoRatchuts::StartMCBMotion()
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

bool StratoRatchuts::ScheduleProfiles()
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

void StratoRatchuts::AddMCBTM()
{
    // make sure it's the correct size
    if (mcbComm.binary_rx.bin_length != MOTION_TM_SIZE) {
        log_error("invalid motion TM size");
        return;
    }

    // if not in real-time mode, add the sync and time
    if (!pibConfigs.real_time_mcb.Read()) {
        // sync byte        
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) 0xA5;
                
        // tenths of seconds since start
        uint16_t elapsed_time = (uint16_t)((millis() - profile_start) / 100);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time >> 8);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (elapsed_time & 0xFF);
    }

    // add each byte of data to the message
    for (int i = 0; i < MOTION_TM_SIZE; i++) {
        MCB_TM_buffer[MCB_TM_buffer_idx++] = mcbComm.binary_rx.bin_buffer[i];
    }

    // if real-time mode, send the TM packet
    if (pibConfigs.real_time_mcb.Read()) {
        snprintf(log_array, LOG_ARRAY_SIZE, "MCB TM Packet %u", ++mcb_tm_counter);
        zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
        zephyrTX.setStateDetails(1, log_array);
        zephyrTX.setStateDetails(2, "");
        zephyrTX.setStateDetails(3, "");
        zephyrTX.setStateFlagValue(1, FINE);
        zephyrTX.setStateFlagValue(2, NOMESS);
        zephyrTX.setStateFlagValue(3, NOMESS);
        ZephyrTXpoke(ZEPHYRTX_TM);
        log_nominal(log_array);
        MCB_TM_buffer_idx = 0; //reser the MCB buffer pointer
    }
}

void StratoRatchuts::NoteProfileStart()
{
    mcb_motion_ongoing = true;
    profile_start = millis();

    if (MOTION_DOCK == mcb_motion || MOTION_IN_NO_LW == mcb_motion) mcb_dock_ongoing = true;

    mcb_tm_counter = 0;
    //zephyrTX.clearTm(); // empty the TM buffer for incoming MCB motion data
    MCB_TM_buffer_idx = 0;
    // Add the start time to the MCB TM Header if not in real-time mode
    if (!pibConfigs.real_time_mcb.Read()) {
        //zephyrTX.addTm((uint32_t) now()); // as a header, add the current seconds since epoch
        uint32_t ProfileStartEpoch  = now();
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 24);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 16);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch >> 8);
        MCB_TM_buffer[MCB_TM_buffer_idx++] = (uint8_t) (ProfileStartEpoch & 0xFF);
    }


}

void StratoRatchuts::SendMCBTM(StateFlag_t state_flag, const char * message)
{
    // use only the first flag to report the motion
    zephyrTX.addTm(MCB_TM_buffer,MCB_TM_buffer_idx);
    zephyrTX.setStateDetails(1, message);
    zephyrTX.setStateDetails(2, "");
    zephyrTX.setStateDetails(3, "");
    zephyrTX.setStateFlagValue(1, state_flag);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    ZephyrTXpoke(ZEPHYRTX_TM);

    //log_nominal(log_array);
    MCB_TM_buffer_idx = 0;
    if (!WriteFileTM("MCB")) {
        log_error("Unable to write MCB TM to SD file");
    }
}


void StratoRatchuts::SendMCBEEPROM()
{
    // the binary buffer has been prepared by the MCBRouter
    zephyrTX.clearTm();
    zephyrTX.addTm(mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length);

    // use only the first flag to preface the contents
    zephyrTX.setStateDetails(1, "MCB EEPROM Contents");
    zephyrTX.setStateDetails(2, "");
    zephyrTX.setStateDetails(3, "");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    ZephyrTXpoke(ZEPHYRTX_TM);

    log_nominal("Sent MCB EEPROM as TM");
}

void StratoRatchuts::SendPIBEEPROM()
{
    // create a buffer from the EEPROM (cheat, and use the preallocated MCBComm Binary RX buffer)
    mcbComm.binary_rx.bin_length = pibConfigs.Bufferize(mcbComm.binary_rx.bin_buffer, MAX_MCB_BINARY);

    if (0 == mcbComm.binary_rx.bin_length) {
        log_error("Unable to bufferize RATCHUTS EEPROM");
        return;
    }

    // prepare the TM buffer
    zephyrTX.clearTm();
    zephyrTX.addTm(mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length);

    // use only the first flag to preface the contents
    zephyrTX.setStateDetails(1, "RATCHUTSEEPROM");
    zephyrTX.setStateDetails(2, "");
    zephyrTX.setStateDetails(3, "");
    zephyrTX.setStateFlagValue(1, FINE);
    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    // send as TM
    TM_ack_flag = NO_ACK;
    ZephyrTXpoke(ZEPHYRTX_TM);

    log_nominal("Sent PIB EEPROM as TM");
}

void StratoRatchuts::SendRPUREPORT(uint8_t packet_num)
{
    uint16_t num_records = puComm.binary_rx.bin_length / RPU_RECORD_BYTES;

    zephyrTX.setStateDetails(1, "RPUREPORT");

    snprintf(log_array, LOG_ARRAY_SIZE, "%u RPURecords", num_records);
    zephyrTX.setStateDetails(2, log_array);

    if (0 < snprintf(log_array, LOG_ARRAY_SIZE, "PU TM: %u.%u, %lu, %0.4f, %0.4f, %0.1f", pibConfigs.profile_id.Read(), packet_num, pu_last_status, profile_start_latitude, profile_start_longitude, profile_start_altitude)) {
        zephyrTX.setStateDetails(3, log_array);
        zephyrTX.setStateFlagValue(1, FINE);
    } else {
        zephyrTX.setStateDetails(3, "PU Profile Record: unable to add status info");
        zephyrTX.setStateFlagValue(1, WARN);
    }

    zephyrTX.setStateFlagValue(2, NOMESS);
    zephyrTX.setStateFlagValue(3, NOMESS);

    TM_ack_flag = NO_ACK;
    ZephyrTXpoke(ZEPHYRTX_TM);

    log_nominal(log_array);
}

void StratoRatchuts::PUDock()
{
    pibConfigs.pu_docked.Write(true);
    digitalWrite(PU_PWR_ENABLE, HIGH);
}

void StratoRatchuts::PUUndock()
{
    pibConfigs.pu_docked.Write(false);
    digitalWrite(PU_PWR_ENABLE, LOW);
}

void StratoRatchuts::PUStartProfile()
{
    // Save the starting lat/lon/alt to include in the profile TMs
    profile_start_latitude = zephyrRX.zephyr_gps.latitude;
    profile_start_longitude = zephyrRX.zephyr_gps.longitude;
    profile_start_altitude = zephyrRX.zephyr_gps.altitude;

    // Enable RPU MEASURE mode with the configured measurement parameters
    puComm.TX_GoMeasure(pibConfigs.rpu_meas_duration.Read(), pibConfigs.rpu_meas_rate.Read(),
                        pibConfigs.rpu_bat_temp.Read(),
                        pibConfigs.rpu_enable_ROPC.Read(), pibConfigs.rpu_enable_TDLAS.Read(),
                        pibConfigs.rpu_enable_TSEN.Read(), pibConfigs.rpu_enable_RS41.Read());

    pibConfigs.profile_id.Write(pibConfigs.profile_id.Read() + 1);
}

void StratoRatchuts::ReadAnalog()
{
    PU_Ir_mon = analogRead(IMON_PU) * (3.000/4095.0)*5000.0; //12 bit, 3V ref, 0.2 Ohm curent shunt to mA
    PU_Ibts_mon = analogRead(IMON_PU_BTS) * (3.000/4095.0); //just return current proportional voltage from BTS
    Vmon_input = analogRead(VMON_15V) * (3.000/4095.0)* (1.13 + 10)/1.13; // 1.13K/10K v divider
    Vmon_3v3 = analogRead(VMON_3V3) * (3.000/4095.0) *2.0 ; // 10k/10k v divider
    MonDo_I_mon = ((1.65-(3.0 * float(analogRead(IMON_MONDO)) /4096.0))/.044); //MonDo input current monitor ACS&1240LLCBTR-030B3
}