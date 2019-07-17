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
    FL_GPS_WAIT,
    FL_START_REEL_OUT,
    FL_START_REEL_IN,
    FL_MONITOR_MOTION,
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
    // todo: draw out flight mode state machine
    switch (inst_substate) {
    case FL_ENTRY:
        // perform setup
        log_nominal("Entering FL");
        inst_substate = FL_GPS_WAIT;
        break;
    case FL_GPS_WAIT:
        // wait for the first GPS message from Zephyr to set the time before moving on
        log_debug("Waiting on GPS time");
        if (time_valid) {
            inst_substate = FL_IDLE;
        }
        break;
    case FL_IDLE:
        if (CheckAction(COMMAND_REEL_IN)) {
            inst_substate = FL_START_REEL_IN;
        } else if (CheckAction(COMMAND_REEL_OUT)) {
            inst_substate = FL_START_REEL_OUT;
        }
        log_debug("FL Idle");
        break;
    case FL_START_REEL_IN:
        mcbTX.retractX(retract_length); // todo: verification
        inst_substate = FL_MONITOR_MOTION;
        break;
    case FL_START_REEL_OUT:
        mcbTX.deployX(deploy_length); // todo: verification
        inst_substate = FL_MONITOR_MOTION;
        break;
    case FL_MONITOR_MOTION:
        // todo: what should be monitored? Just check for MCB messages?

        if (CheckAction(COMMAND_MOTION_STOP)) {
            // todo: how to verify, command already sent in router, will a motion finished be sent?
        }

        if (mcb_motion_finished) {
            inst_substate = FL_IDLE;
        }
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