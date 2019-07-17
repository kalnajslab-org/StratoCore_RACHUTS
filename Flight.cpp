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
    FL_ERROR_LANDING,
    FL_ERROR_LOOP,
    
    FL_SHUTDOWN = MODE_SHUTDOWN,
    FL_EXIT = MODE_EXIT
};

char log_array[101] = {0};

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
        snprintf(log_array, 101, "Retracting %0.1f revs", retract_length);
        ZephyrLogFine(log_array);
        mcb_motion_finished = false;
        mcbTX.retractX(retract_length); // todo: verification
        inst_substate = FL_MONITOR_MOTION;
        break;
    case FL_START_REEL_OUT:
        snprintf(log_array, 101, "Deploying %0.1f revs", deploy_length);
        ZephyrLogFine(log_array);
        mcb_motion_finished = false;
        mcbTX.deployX(deploy_length); // todo: verification
        inst_substate = FL_MONITOR_MOTION;
        break;
    case FL_MONITOR_MOTION:
        // todo: what should be monitored? Just check for MCB messages?

        if (CheckAction(COMMAND_MOTION_STOP)) {
            // todo: verification of motion stop
            ZephyrLogFine("Commanded motion stop");
            inst_substate = FL_IDLE;
            break;
        }

        if (mcb_motion_finished) {
            ZephyrLogFine("Motion complete");
            inst_substate = FL_IDLE;
        }
        break;
    case FL_ERROR_LANDING:
        // generic error state for flight mode to go to if any error is detected
        // this state can make sure the ground is informed, and go to the error looop to wait for ground intervention
        // before setting this substate, a ZephyrLogCrit should be sent
        inst_substate = FL_ERROR_LOOP;
        break;
    case FL_ERROR_LOOP:
        // wait for ground
        // todo: add exit condition
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
        ZephyrLogCrit("Unknown substate in FL");
        inst_substate = FL_ERROR_LANDING;
        break;
    }
}