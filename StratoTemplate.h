/*
 *  StratoTemplate.h
 *  Author:  Alex St. Clair
 *  Created: June 2019
 *  
 *  This file declares an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as both a template and test
 *  class for inheriting from the StratoCore.
 */

#ifndef STRATOTEMPLATE_H
#define STRATOTEMPLATE_H

#include "StratoCore.h"
#include "TemplateBufferGuard.h"

// for testing purposes, use LPC
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

class StratoTemplate : public StratoCore {
public:
    StratoTemplate();
    ~StratoTemplate() { };

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

#endif /* STRATOTEMPLATE_H */