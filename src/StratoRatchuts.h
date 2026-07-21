/*
 *  StratoRatchuts.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  Major update: Decemvber 2024 for T4.1, and MonDo Rev E
 * 
 *  This file declares an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the RACHuTS Profiler Interface Board, or PIB.
 */

#ifndef STRATORATCHUTS_H
#define STRATORATCHUTS_H

#include "StratoCore.h"
#include "PIBHardware.h"
//#include "PIBBufferGuard.h" //this is not needed for Teensy 4.1 as buffer size is set in user code
#include "PIBConfigs.h"
#include "MCBComm.h"
#include "RPUComm.h"
#include "LoRa.h"

#define INSTRUMENT   RACHUTS
#define ZEPHYR_SERIAL_BUFFER_SIZE 4096
#define MCB_SERIAL_BUFFER_SIZE    4096
// Must exceed the largest RPU_PROFILE_RECORD frame (PU_BUFFER_SIZE payload plus
// framing/checksum) so a full record batch buffers without UART RX overflow.
#define PU_SERIAL_BUFFER_SIZE     16384

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      3

#define MCB_RESEND_TIMEOUT      10
#define PU_RESEND_TIMEOUT       10
#define ZEPHYR_RESEND_TIMEOUT   60

#define RETRY_DOCK_LENGTH   2.0f

#define MCB_BUFFER_SIZE     MAX_MCB_BINARY
#define PU_BUFFER_SIZE      8192

//LoRa Settings
#define FREQUENCY 868E6
#define BANDWIDTH 250E3
#define SF 9
#define RF_POWER 19
#define LORA_TM_TIMEOUT 600

// todo: update naming to be more unique (ie. ACT_ prefix)
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,

    // scheduled actions
    SEND_IMR,
    RESEND_SAFETY,
    RESEND_MCB_LP,
    RESEND_RA,
    RESEND_MOTION_COMMAND,
    RESEND_TM,
    RESEND_PU_CHECK,
    RESEND_PU_RECORD,
    RESEND_PU_WARMUP,
    RESEND_PU_GOPROFILE,
    RESEND_FULL_RETRACT,

    // exit the error state (ground command only)
    EXIT_ERROR_STATE,

    // internal actions
    ACTION_REEL_OUT,
    ACTION_REEL_IN,
    ACTION_IN_NO_LW,
    ACTION_DOCK,
    ACTION_MOTION_STOP,
    ACTION_BEGIN_PROFILE,
    ACTION_END_DWELL,
    ACTION_CHECK_PU,
    ACTION_END_WARMUP,
    ACTION_END_PREPROFILE,
    ACTION_OFFLOAD_PU,
    ACTION_MOTION_TIMEOUT,
    ACTION_END_DOCK_WAIT,

    // Multi-action commands
    COMMAND_REDOCK,    // reel out, reel in (no lw), check PU
    COMMAND_MANUAL_PROFILE,
    COMMAND_DOCKED_PROFILE,

    // used for tracking
    NUM_ACTIONS
};

enum MCBMotion_t : uint8_t {
    NO_MOTION,
    MOTION_REEL_IN,
    MOTION_REEL_OUT,
    MOTION_DOCK,
    MOTION_IN_NO_LW
};

// Specifies which ZephyrTX member function to call in ZephyrTXpoke().
enum ZephyrTXMsgType_t : uint8_t {
    ZEPHYRTX_TM,
    ZEPHYRTX_S,
    ZEPHYRTX_IMR,
    ZEPHYRTX_RA
};

class StratoRatchuts : public StratoCore {
public:
    StratoRatchuts();
    ~StratoRatchuts() { };

    // called before the main loop begins
    void InstrumentSetup();

    // called at the end of each main loop
    void InstrumentLoop();

    // called in each main loop
    void RunMCBRouter();
    void RunPURouter();
    void LoRaRX();
    void LoRaInit();
    void SendRPUSTATUS(const String& json, const String& source);

    // Send a text TM (StateMess1 = "RATCHUTSTEXT") with the given StateFlag1
    // (FINE/WARN/CRIT). Unlike the base ZephyrLog*(), this routes through
    // ZephyrTXpoke() so the transceiver is woken first, and it also logs locally.
    void SendTextTM(const char * message, StateFlag_t flag);

    // Wake up the MAX3381 serial transceiver by sending a blank character to
    // ZEPHYR_SERIAL before calling the specified ZephyrTX member function. The
    // MAX3381 has a 30-second inactivity timeout, after which it powers down and
    // can drop the first transmitted byte.
    void ZephyrTXpoke(ZephyrTXMsgType_t msg_type);

private:
    // internal serial interface objects for the MCB and PU
    MCBComm mcbComm;
    RPUComm puComm;

    // EEPROM interface object
    PIBConfigs pibConfigs;

    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();

    // Flight mode subsets (in Flight.cpp)
    void AutonomousFlight();
    void ManualFlight();

    // Flight states under autonomous or manual (each in own .cpp file)
    // when starting the state, call with restart_state = true
    // then call with restart_state = false until the function returns true meaning it's completed
    bool Flight_CheckPU(bool restart_state);
    bool Flight_Profile(bool restart_state);
    bool Flight_ReDock(bool restart_state);
    bool Flight_PUOffload(bool restart_state);
    bool Flight_ManualMotion(bool restart_state);
    bool Flight_DockedProfile(bool restart_state);

    // Telcommand handler - returns ack/nak
    bool TCHandler(Telecommand_t telecommand);

    // Guard for manual-only TCs: returns true if in manual flight mode,
    // otherwise sets the TC-ack detail (msg3) + flag naming the command and the
    // required mode, and returns false (so the caller can break out).
    bool RequireManualFlight(const char * cmd, String & msg3, StateFlag_t & flag);

    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);

    // Safely check and clear action flags
    bool CheckAction(uint8_t action);

    // Correctly set an action flag
    void SetAction(uint8_t action);

    // Monitor the action flags and clear old ones
    void WatchFlags();

    // Handle messages from the MCB (in MCBRouter.cpp)
    void HandleMCBASCII();
    void HandleMCBAck();
    void HandleMCBBin();
    void HandleMCBString();
    uint8_t binary_mcb[MCB_BUFFER_SIZE];

    // Handle messages from the PU (in PURouter.cpp)
    void HandlePUASCII();
    void HandlePUAck();
    void HandlePUBin();
    void HandlePUString();
    uint8_t binary_pu[PU_BUFFER_SIZE];

    // Start any type of MCB motion
    bool StartMCBMotion();

    // Schedule profiles in autonomous mode
    bool ScheduleProfiles();

    // Add an MCB motion TM packet to the binary TM buffer
    void AddMCBTM();

    // Set variables and TM buffer after a profile starts
    void NoteProfileStart();

    // Send a telemetry packet with MCB binary info
    void SendMCBTM(const char * TMname, StateFlag_t state_flag, const char * message);

    // Send a telemetry packet with EEPROM contents
    void SendMCBEEPROM();
    void SendPIBEEPROM();

    void SendRPUREPORT(uint8_t packet_num);

    // call every time the known state of the PU changes
    void PUDock();
    void PUUndock();

    // PU start profile command generation and transmit
    void PUStartProfile();

    // Read the analog channels on the PIB
    void ReadAnalog();

    // 2-letter code of the current StratoCore mode (SB/FL/LP/SA/EF), set at the
    // top of each mode function. Used in the RPUSTATUS TM StateDetails.
    const char * mode_code = "SB";

    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false

    // track the flight mode (autonomous/manual)
    bool autonomous_mode = false;

    // flags for MCB state tracking
    bool mcb_low_power = false;
    bool mcb_motion_ongoing = false;
    bool mcb_dock_ongoing = false;
    uint32_t max_profile_seconds = 0;
    bool mcb_reeling_in = false;
    uint16_t mcb_tm_counter = 0;
    float reel_pos = 0.0;


    // flags for PU state tracking
    bool record_received = false;
    bool pu_no_more_records = false;
    bool pu_measure = false;
    bool pu_preprofile = false;
    bool check_pu_success = false;

    // tracks the number of profiles remaining in autonomous mode and if they're scheduled
    uint8_t profiles_remaining = 0;
    bool profiles_scheduled = false;

    // uint32_t start time of the current profile in millis
    uint32_t profile_start = 0;

    // variables to hold start lat/lon/alt of each profile
    float profile_start_latitude = 0.0;
    float profile_start_longitude = 0.0;
    float profile_start_altitude = 0.0;

    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;

    // current profile parameters
    float deploy_length = 0.0f;
    float retract_length = 0.0f;
    float dock_length = 0.0f;

    // current docked profile duration
    uint16_t docked_profile_time = 0;

    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};
    uint8_t MCB_TM_buffer[8192] = {0};
    uint16_t MCB_TM_buffer_idx = 0;

    // PU status information
    uint32_t pu_last_status = 0;        // RACHUTS-local time of last received RPU status
    String pu_status_json;              // raw JSON status string from RPU
    bool pu_status_received = false;    // set when a fresh RPU_STATUS is received, cleared by Flight_CheckPU

    uint8_t eeprom_buffer[256];

    //Variables for LoRa TMs and Status strings
    bool Send_LoRa_TM = true;
    bool Send_LoRa_status = true;
    char LoRa_RX_buffer[256] = {0};
    char LoRa_PU_status[256] = {0};
    
    uint8_t LoRa_TM_buffer[8192] = {0};
    uint16_t LoRa_TM_buffer_idx = 0;
    uint16_t pu_tm_counter = 0;
    long LoRa_rx_time = 0;

    //variables for PIB house keeping values
    float PU_Ir_mon = 0.0;
    float PU_Ibts_mon = 0.0;
    float Vmon_input = 0.0;
    float Vmon_3v3 = 0.0;
    float MonDo_I_mon = 0.0;
};

#endif /* STRATORATCHUTS_H */