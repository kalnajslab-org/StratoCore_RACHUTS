/*
 *  LowPower.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  This file implements the RACHuTS low power mode.
 */

#include "StratoPIB.h"

enum LPStates_t : uint8_t {
    LP_ENTRY = MODE_ENTRY,
    
    // add any desired states between entry and shutdown
    LP_ALERT_MCB,
    LP_CHECK_MCB,
    LP_LOOP,
    
    LP_SHUTDOWN = MODE_SHUTDOWN,
    LP_EXIT = MODE_EXIT
};

void StratoPIB::LowPowerMode()
{
    switch (inst_substate) {
    case LP_ENTRY:
        // perform setup
        log_nominal("Entering LP");
        inst_substate = LP_ALERT_MCB;
        break;
    case LP_ALERT_MCB:
        log_nominal("Commanding MCB low power");
        mcbTX.goLow();
        scheduler.AddAction(RESEND_MCB_LP, 60);
        inst_substate = LP_CHECK_MCB;
        break;
    case LP_CHECK_MCB:
        log_debug("Waiting on MCB LP ack");
        if (mcb_low_power) {
            mcb_low_power = false;
            inst_substate = LP_LOOP;
        } else if (CheckAction(RESEND_MCB_LP)) {
            inst_substate = LP_ALERT_MCB;
        }
        break;
    case LP_LOOP:
        // nominal ops
        log_debug("LP loop");
        break;
    case LP_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in LP");
        break;
    case LP_EXIT:
        // perform cleanup
        log_nominal("Exiting LP");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in LP");
        inst_substate = LP_ENTRY; // reset
        break;
    }
}