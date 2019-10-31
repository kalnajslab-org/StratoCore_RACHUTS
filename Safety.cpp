/*
 *  Safety.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file implements the RACHuTS safety mode.
 */

#include "StratoPIB.h"

enum SAStates_t : uint8_t {
    SA_ENTRY = MODE_ENTRY,

    // add any desired states between entry and shutdown
    SA_SEND_FULL_RETRACT,
    SA_VERIFY_FULL_RETRACT,
    SA_MONITOR_FULL_RETRACT,
    SA_SEND_MCB_LP,
    SA_VERIFY_MCB_LP,
    SA_LOOP,
    SA_SEND_S,
    SA_ACK_WAIT,

    SA_ERROR_LANDING = MODE_ERROR,
    SA_SHUTDOWN = MODE_SHUTDOWN,
    SA_EXIT = MODE_EXIT
};

void StratoPIB::SafetyMode()
{
    switch (inst_substate) {
    case SA_ENTRY:
        // perform setup
        log_nominal("Entering SA");
        inst_substate = SA_SEND_FULL_RETRACT;
        break;
    case SA_SEND_FULL_RETRACT:
        mcb_reeling_in = false;
        mcb_motion_ongoing = true;
        mcbComm.TX_ASCII(MCB_FULL_RETRACT);
        scheduler.AddAction(RESEND_FULL_RETRACT, MCB_RESEND_TIMEOUT);
        inst_substate = SA_VERIFY_FULL_RETRACT;
        break;
    case SA_VERIFY_FULL_RETRACT:
        if (mcb_reeling_in) {
            log_nominal("MCB performing full retract");
            inst_substate = SA_MONITOR_FULL_RETRACT;
        }

        if (CheckAction(RESEND_FULL_RETRACT)) {
            inst_substate = SA_SEND_FULL_RETRACT;
        }
        break;
    case SA_MONITOR_FULL_RETRACT:
        if (!mcb_motion_ongoing) {
            log_nominal("MCB full retract appears complete");
            inst_substate = SA_SEND_MCB_LP;
        }
        break;
    case SA_SEND_MCB_LP:
        mcb_low_power = false;
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
        scheduler.AddAction(RESEND_MCB_LP, MCB_RESEND_TIMEOUT);
        inst_substate = SA_VERIFY_MCB_LP;
        break;
    case SA_VERIFY_MCB_LP:
        if (mcb_low_power) {
            log_nominal("MCB in low power for safety");
            inst_substate = SA_SEND_S;
        }

        if (CheckAction(RESEND_MCB_LP)) {
            mcbComm.TX_ASCII(MCB_GO_LOW_POWER);
            inst_substate = SA_SEND_S; // actually just skip to sending safety
        }
        break;
    case SA_SEND_S:
        log_nominal("Sending safety message");
        digitalWrite(SAFE_PIN, HIGH);
        zephyrTX.S();
        scheduler.AddAction(RESEND_SAFETY, ZEPHYR_RESEND_TIMEOUT);
        inst_substate = SA_ACK_WAIT;
        break;
    case SA_ACK_WAIT:
        log_debug("Waiting on safety ack");
        // check if the ack has been received
        if (S_ack_flag == ACK) {
            // clear the ack flag and go to the loop
            S_ack_flag = NO_ACK;
            inst_substate = SA_LOOP;
        } else if (S_ack_flag == NAK) {
            // just clear the ack flag -- a resend is already scheduled
            S_ack_flag = NO_ACK;
        }

        // if a minute has passed, resend safety
        if (CheckAction(RESEND_SAFETY)) {
            inst_substate = SA_SEND_S;
        }

        break;
    case SA_LOOP:
        // nominal ops
        log_debug("SA loop");
        digitalWrite(SAFE_PIN, HIGH);
        break;
    case SA_ERROR_LANDING:
        log_debug("SA error");
        break;
    case SA_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in SA");
        break;
    case SA_EXIT:
        // perform cleanup
        digitalWrite(SAFE_PIN, LOW);
        log_nominal("Exiting SA");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in SA");
        inst_substate = SA_ENTRY; // reset
        break;
    }
}