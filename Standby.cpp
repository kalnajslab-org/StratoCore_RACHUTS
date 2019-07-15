/*
 *  Standby.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  This file implements the RACHuTS standby mode.
 */

#include "StratoPIB.h"

enum SBStates_t : uint8_t {
    SB_ENTRY = MODE_ENTRY,
    
    // add any desired states between entry and shutdown
    SB_LOOP,
    
    SB_SHUTDOWN = MODE_SHUTDOWN,
    SB_EXIT = MODE_EXIT
};

void StratoPIB::StandbyMode()
{
    switch (inst_substate) {
    case SB_ENTRY:
        log_nominal("Entering SB");

        // send mode request in first loop
        scheduler.AddAction(SEND_IMR, 0);

        inst_substate = SB_LOOP;
        break;
    case SB_LOOP:
        // nominal ops
        log_debug("SB loop");

        // send a mode request if time, and schedule the next
        if (CheckAction(SEND_IMR)) {
            log_nominal("Sending mode request to OBC");
            zephyrTX.IMR();
            scheduler.AddAction(SEND_IMR, 60);
        }
        break;
    case SB_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in SB");
        break;
    case SB_EXIT:
        // perform cleanup
        log_nominal("Exiting SB");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in SB");
        inst_substate = SB_ENTRY; // reset
        break;
    }
}