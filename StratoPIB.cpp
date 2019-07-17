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
    , mcbTX(&MCB_SERIAL, &Serial)
    , mcbRX(&MCB_SERIAL, &Serial, DIB) // note: MCB expects DIB due to hard-coded Reader/Writer functions
{
    mcbTX.setDevId("DIB"); // note: MCB expects DIB due to hard-coded Reader/Writer functions
    waiting_mcb_messages = 0;
    mcb_low_power = false;
    mcb_motion_finished = false;
}

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
}

void StratoPIB::InstrumentLoop()
{
    WatchFlags();
}

// The telecommand handler must return ACK/NAK
bool StratoPIB::TCHandler(Telecommand_t telecommand)
{
    String dbg_msg = "";

    switch (telecommand) {
    case DEPLOYx:
        deploy_length = mcbParam.deployLen;
        SetAction(COMMAND_REEL_OUT); // will be ignored if wrong mode
        break;
    case DEPLOYv:
        mcbTX.deployV(mcbParam.deployVel); // todo: verification + ack
        break;
    case DEPLOYa:
        mcbTX.deployA(mcbParam.deployAcc); // todo: verification + ack
        break;
    case RETRACTx:
        retract_length = mcbParam.retractLen;
        SetAction(COMMAND_REEL_IN); // will be ignored if wrong mode
        break;
    case RETRACTv:
        mcbTX.retractV(mcbParam.retractVel); // todo: verification + ack
        break;
    case RETRACTa:
        mcbTX.retractA(mcbParam.retractAcc); // todo: verification + ack
        break;
    case FULLRETRACT:
        // todo: determine implementation
        break;
    // case DOCKx;
    //     break;
    case CANCELMOTION:
        mcbTX.cancelMotion(); // no matter what, attempt to send (irrespective of mode)
        SetAction(COMMAND_MOTION_STOP);
        break;
    default:
        log_error("Unknown TC received");
        // error case here
        break;
    }
    return true;
}

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

void StratoPIB::TakeMCBByte(uint8_t new_byte)
{
    mcbRX.putChar(new_byte);
    if (new_byte == 3) { // EOF char
        waiting_mcb_messages++;
    }
}

void StratoPIB::RunMCBRouter()
{
    if (waiting_mcb_messages) {
        waiting_mcb_messages--;
        mcbRX.igetNew();

        if (mcbRX.dataValid()) {
            switch (mcb_message) {
            case LOW_POWER_ACK:
                log_nominal("MCB in low power");
                mcb_low_power = true;
                break;
            case MOTION_FINISHED:
                log_nominal("MCB motion finished");
                mcb_motion_finished = true;
                break;
            default:
                log_error("Unknown MCB message received");
                break;
            }
        }
    }
}