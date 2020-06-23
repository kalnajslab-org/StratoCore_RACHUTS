/*
 *  StratoPIB.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file declares an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as the overarching class
 *  for the RACHuTS Profiler Interface Board, or PIB.
 */

#ifndef STRATOPIB_H
#define STRATOPIB_H

#include "StratoCore.h"
#include "PIBHardware.h"
#include "PIBBufferGuard.h"
#include "PIBConfigs.h"
#include "MCBComm.h"
#include "PUComm.h"

#define INSTRUMENT      RACHUTS

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      3

#define MCB_RESEND_TIMEOUT      10
#define PU_RESEND_TIMEOUT       10
#define ZEPHYR_RESEND_TIMEOUT   60

#define RETRY_DOCK_LENGTH   2.0f

#define MCB_BUFFER_SIZE     MAX_MCB_BINARY
#define PU_BUFFER_SIZE      8192

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
    RESEND_PU_TSEN,
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
    ACTION_REQUEST_TSEN, // send the TSEN request
    ACTION_END_WARMUP,
    ACTION_END_PREPROFILE,
    ACTION_OVERRIDE_TSEN, // if TSEN in manual, override for command
    ACTION_OFFLOAD_PU,
    ACTION_MOTION_TIMEOUT,
    ACTION_END_DOCK_WAIT,

    // Multi-action commands
    COMMAND_REDOCK,    // reel out, reel in (no lw), check PU
    COMMAND_SEND_TSEN, // check PU, request TSEN, send TM
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

struct PUStatus_t {
    uint32_t last_status;
    uint32_t time;
    float v_battery;
    float i_charge;
    float therm1;
    float therm2;
    uint8_t heater_stat;
};

class StratoPIB : public StratoCore {
public:
    StratoPIB();
    ~StratoPIB() { };

    // called before the main loop begins
    void InstrumentSetup();

    // called at the end of each main loop
    void InstrumentLoop();

    // called in each main loop
    void RunMCBRouter();
    void RunPURouter();

private:
    // internal serial interface objects for the MCB and PU
    MCBComm mcbComm;
    PUComm puComm;

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
    bool Flight_TSEN(bool restart_state);
    bool Flight_ManualMotion(bool restart_state);
    bool Flight_DockedProfile(bool restart_state);

    // Telcommand handler - returns ack/nak
    void TCHandler(Telecommand_t telecommand);

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
    void SendMCBTM(StateFlag_t state_flag, const char * message);

    // Send a telemetry packet with EEPROM contents
    void SendMCBEEPROM();
    void SendPIBEEPROM();

    // send a telemetry packet with PU TSEN or Profile Record info
    void SendTSENTM();
    void SendProfileTM(uint8_t packet_num);

    // sets an action flag every ten minutes aligned with the hour
    void CheckTSEN();

    // call every time the known state of the PU changes
    void PUDock();
    void PUUndock();

    // PU start profile command generation and transmit
    void PUStartProfile();

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

    // flags for PU state tracking
    bool record_received = false;
    bool tsen_received = false;
    bool pu_no_more_records = false;
    bool pu_warmup = false;
    bool pu_profile = false;
    bool pu_preprofile = false;
    bool check_pu_success = false;

    // tracks the number of profiles remaining in autonomous mode and if they're scheduled
    uint8_t profiles_remaining = 0;
    bool profiles_scheduled = false;

    // uint32_t start time of the current profile in millis
    uint32_t profile_start = 0;

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

    // PU status information
    PUStatus_t pu_status = {0};

    uint8_t eeprom_buffer[256];
};

#endif /* STRATOPIB_H */