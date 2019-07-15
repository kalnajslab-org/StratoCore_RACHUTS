/*
 *  StratoTemplate.cpp
 *  Author:  Alex St. Clair
 *  Created: June 2019
 *  
 *  This file implements an Arduino library (C++ class) that inherits
 *  from the StratoCore class. It serves as both a template and test
 *  class for inheriting from the StratoCore.
 */

#include "StratoTemplate.h"

StratoTemplate::StratoTemplate()
    : StratoCore(&ZEPHYR_SERIAL, INSTRUMENT)
{
}

void StratoTemplate::InstrumentSetup()
{
    // for LPC RS232 transceiver for testing
    pinMode(29, OUTPUT);
    pinMode(30, OUTPUT);
    digitalWrite(29, HIGH);
    digitalWrite(30, HIGH);
}

void StratoTemplate::InstrumentLoop()
{
    WatchFlags();
}

// The telecommand handler must return ACK/NAK
bool StratoTemplate::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";

    switch (telecommand) {
    case SETSAMPLE: // example for LPC (TC: 101)
        dbg_msg = "Received set sample: " + String(lpcParam.samples);
        log_nominal(dbg_msg.c_str());
        break;
    default:
        log_error("Unknown TC received");
        // error case here
        break;
    }
    return true;
}

void StratoTemplate::ActionHandler(uint8_t action)
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

bool StratoTemplate::CheckAction(uint8_t action)
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

void StratoTemplate::WatchFlags()
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