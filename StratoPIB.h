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
#include "MCBComm.h"

#define INSTRUMENT      RACHUTS

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2

// todo: update naming to be more unique (ie. ACT_ prefix)
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,

    // scheduled actions
    SEND_IMR,
    RESEND_SAFETY,
    RESEND_MCB_LP,
    RESEND_RA,
    RESEND_MOTION_COMMAND,

    // internal command actions
    COMMAND_REEL_OUT,
    COMMAND_REEL_IN,
    COMMAND_DOCK,
    COMMAND_MOTION_STOP,

    // used for tracking
    NUM_ACTIONS
};

enum MCBMotion_t : uint8_t {
    NO_MOTION,
    MOTION_REEL_IN,
    MOTION_REEL_OUT,
    MOTION_DOCK,
    MOTION_UNDOCK
};

class StratoPIB : public StratoCore {
public:
    StratoPIB();
    ~StratoPIB() { };

    // called before the loop begins
    void InstrumentSetup();

    // called at the end of each loop
    void InstrumentLoop();

    // called in each loop
    void RunMCBRouter();

private:
    // internal serial interface object for the MCB
    MCBComm mcbComm;

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

    // Start any type of MCB motion
    bool StartMCBMotion();

    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false

    // track the flight mode (autonomous/manual)
    bool autonomous_mode = false;

    // flags for MCB state tracking
    bool mcb_low_power = false;
    bool mcb_motion_ongoing = false;

    // tracks the current type of motion
    MCBMotion_t mcb_motion = NO_MOTION;

    // tracks if a resend of any message has already been attempted
    bool resend_attempted = false;

    // profile parameters
    float deploy_length = 0.0f;
    float deploy_velocity = 250.0f;
    float retract_length = 0.0f;
    float retract_velocity = 250.0f;
    float dock_length = 0.0f;
    float dock_velocity = 80.0f;

    // array of error values for MCB motion fault
    uint16_t motion_fault[8] = {0};

    // keep a statically allocated array for creating up to 100 char TM state messages
    char log_array[101] = {0};
};

#endif /* STRATOPIB_H */