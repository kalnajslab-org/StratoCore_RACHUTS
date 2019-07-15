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
#include "PIBBufferGuard.h"

#define ZEPHYR_SERIAL   Serial2 // LPC
#define INSTRUMENT      LPC

// number of loops before a flag becomes stale and is reset
#define FLAG_STALE      2

// todo: perhaps more creative/useful enum here by mode with separate arrays?
enum ScheduleAction_t : uint8_t {
    NO_ACTION = NO_SCHEDULED_ACTION,
    SEND_IMR,
    RESEND_SAFETY,
    NUM_ACTIONS
};

class StratoPIB : public StratoCore {
public:
    StratoPIB();
    ~StratoPIB() { };

    // called before the loop begins
    void InstrumentSetup();

    // called at the end of each loop
    void InstrumentLoop();

private:
    // Mode functions (implemented in unique source files)
    void StandbyMode();
    void FlightMode();
    void LowPowerMode();
    void SafetyMode();
    void EndOfFlightMode();

    // Telcommand handler - returns ack/nak
    bool TCHandler(Telecommand_t telecommand);

    // Action handler for scheduled actions
    void ActionHandler(uint8_t action);

    // Safely check and clear action flags
    bool CheckAction(uint8_t action);

    // Monitor the action flags and clear old ones
    void WatchFlags();

    ActionFlag_t action_flags[NUM_ACTIONS] = {{0}}; // initialize all flags to false
};

#endif /* STRATOPIB_H */