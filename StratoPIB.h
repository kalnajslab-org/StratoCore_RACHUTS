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
#include "PIBStorage.h"
#include "MCBComm.h"
#include "PUComm.h"

#define INSTRUMENT      RACHUTS

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2

#define MCB_RESEND_TIMEOUT      10
#define PU_RESEND_TIMEOUT       10
#define ZEPHYR_RESEND_TIMEOUT   60

#define LOG_ARRAY_SIZE  101

#define RETRY_DOCK_LENGTH   2.0f

#define MCB_BUFFER_SIZE     50
#define PU_BUFFER_SIZE      8192

#define TSEN_READ_PERIOD    870 // 15 min minus 30 seconds of overhead

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

    // exit the error state (ground command only)
    EXIT_ERROR_STATE,

    // internal actions
    ACTION_REEL_OUT,
    ACTION_REEL_IN,
    ACTION_DOCK,
    ACTION_MOTION_STOP,
    ACTION_BEGIN_PROFILE,
    ACTION_END_DWELL,
    ACTION_UNDOCK,
    ACTION_CHECK_PU,
    ACTION_REQUEST_TSEN, // send the TSEN request

    // Multi-action commands
    COMMAND_REDOCK,    // reel out, reel in (no lw), check PU
    COMMAND_SEND_TSEN, // check PU, request TSEN, send TM

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
    PIBStorage pibStorage;

    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();

    // Flight mode subsets (in Flight.cpp)
    void AutonomousFlight();
    void ManualFlight();

    // Telcommand handler - returns ack/nak
    bool TCHandler(Telecommand_t telecommand);

    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);

    // Safely check and clear action flags
    bool CheckAction(uint8_t action);

    // Correctly set an action flag
    void SetAction(uint8_t action);

    // Monitor the action flags and clear old ones
    void WatchFlags();

    // Handle messages from the MCB
    void HandleMCBASCII();
    void HandleMCBAck();
    void HandleMCBBin();
    uint8_t binary_mcb[MCB_BUFFER_SIZE];

    // Handle messages from the PU
    void HandlePUASCII();
    void HandlePUAck();
    void HandlePUBin();
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

    // send a telemetry packet with PU TSEN info
    void SendTSENTM();

    // schedule TSEN packets every 15 minutes synchronized with the hour
    bool ScheduleNextTSEN();

    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false

    // track the flight mode (autonomous/manual)
    bool autonomous_mode = false;

    // flags for MCB state tracking
    bool mcb_low_power = false;
    bool mcb_motion_ongoing = false;
    bool mcb_dock_ongoing = false;

    // flags for PU state tracking
    bool tsen_received = false;
    bool send_pu_status = false;
    bool pu_no_more_records = false;

    // tracks the number of profiles remaining in autonomous mode
    uint8_t profiles_remaining = 0;

    // uint32_t start time of the current profile in millis
    uint32_t profile_start = 0;

    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;

    // tracks if a resend of any message has already been attempted
    bool resend_attempted = false;

    // current profile parameters
    float deploy_length = 0.0f;
    float retract_length = 0.0f;
    float dock_length = 0.0f;

    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};

    // PU status information
    uint32_t PUTime = 0;
    float PUVBattery = 0.0f;
    float PUICharge = 0.0f;
    float PUTherm1T = 0.0f;
    float PUTherm2T = 0.0f;
    uint8_t PUHeaterStat = 0;

    // keep a statically allocated array for creating up to 100 char TM state messages
    char log_array[LOG_ARRAY_SIZE] = {0};
};

#endif /* STRATOPIB_H */