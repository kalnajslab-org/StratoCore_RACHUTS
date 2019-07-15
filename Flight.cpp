/*
 *  Flight.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  This file implements the RACHuTS flight mode.
 */

#include "StratoPIB.h"

enum FLStates_t : uint8_t {
    FL_ENTRY = MODE_ENTRY,
    
    // add any desired states between entry and shutdown
    FL_IDLE,
    FL_PU_CONFIG,
    FL_MAGNETS_OFF,
    FL_UNDOCK,
    FL_WAIT_FOR_PROFILE,
    FL_DOCK,
    FL_READ_PU,
    FL_ERROR,
    
    FL_SHUTDOWN = MODE_SHUTDOWN,
    FL_EXIT = MODE_EXIT
};

// this function is called at the defined rate
//  * when flight mode is entered, it will start in FL_ENTRY state
//  * it is then up to this function to change state as needed by updating the inst_substate variable
//  * on each loop, whichever substate is set will be perfomed
//  * when the mode is changed by the Zephyr, FL_EXIT will automatically be set
//  * it is up to the FL_EXIT logic perform any actions for leaving flight mode
void StratoPIB::FlightMode()
{
    switch (inst_substate) {
    case FL_ENTRY:
        // perform setup
        log_nominal("Entering FL");
        inst_substate = FL_IDLE; // automatically go to idle
        break;
    case FL_IDLE:
        // some logic here to determine when to leave idle and go to PU_CONFIG, e.g.:
        // if (time_for_measurement)
        //    inst_substate = PU_CONFIG
        log_debug("FL Idle");
        break;
    case FL_PU_CONFIG:
        // logic/functions to configure the PU
        inst_substate = FL_MAGNETS_OFF;
        break;
    case FL_MAGNETS_OFF:
        // logic/functions to turn the magnets off
        inst_substate = FL_UNDOCK;
        break;
    case FL_UNDOCK:
        // logic/functions for undocking
        inst_substate = FL_WAIT_FOR_PROFILE;
        break;
    case FL_WAIT_FOR_PROFILE:
        // logic/functions to determine when the profile is over and to move to the next mode
        // any necessary monitoring
        break;
    case FL_DOCK:
        // logic/functions to perform docking and determine what to do next
        break;
    case FL_READ_PU:
        // logic/functions to read and store all of the data from the PU
        // lots of data, so will likely involve polling of read status or something
        break;
    case FL_ERROR:
        // generic error state for flight mode to go to if any error is detected
        // this state can make sure the ground is informed, and wait for ground intervention
        break;
    case FL_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in FL");
        break;
    case FL_EXIT:
        // perform cleanup
        log_nominal("Exiting FL");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in FL");
        inst_substate = FL_ENTRY; // reset
        break;
    }
}