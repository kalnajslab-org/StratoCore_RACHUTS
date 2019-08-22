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
    , mcbComm(&MCB_SERIAL)
{
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
        deploy_velocity = mcbParam.deployVel;
        break;
    case DEPLOYa:
        mcbComm.TX_Out_Acc(mcbParam.deployAcc); // todo: verification + ack
        break;
    case RETRACTx:
        retract_length = mcbParam.retractLen;
        SetAction(COMMAND_REEL_IN); // will be ignored if wrong mode
        break;
    case RETRACTv:
        retract_velocity = mcbParam.retractVel;
        break;
    case RETRACTa:
        mcbComm.TX_In_Acc(mcbParam.retractAcc); // todo: verification + ack
        break;
    case DOCKx:
        dock_length = mcbParam.dockLen;
        SetAction(COMMAND_DOCK); // will be ignored if wrong mode
        break;
    case DOCKv:
        dock_velocity = mcbParam.dockVel;
        break;
    case DOCKa:
        mcbComm.TX_Dock_Acc(mcbParam.dockAcc); // todo: verification + ack
        break;
    case FULLRETRACT:
        // todo: determine implementation
        break;
    case CANCELMOTION:
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION); // no matter what, attempt to send (irrespective of mode)
        SetAction(COMMAND_MOTION_STOP);
        break;
    case SETAUTO:
        if (!mcb_motion_ongoing) {
            autonomous_mode = true;
            inst_substate = MODE_ENTRY; // restart FL in auto
        } else {
            return false;
        }
        break;
    case SETMANUAL:
        if (!mcb_motion_ongoing) {
            autonomous_mode = false;
            inst_substate = MODE_ENTRY; // restart FL in manual
        } else {
            return false;
        }
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

void StratoPIB::RunMCBRouter()
{
    SerialMessage_t rx_msg = mcbComm.RX();

    while (NO_MESSAGE != rx_msg) {
        if (ASCII_MESSAGE == rx_msg) {
            HandleMCBASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandleMCBAck();
        } else {
            log_error("Non-ASCII message from MCB");
        }

        rx_msg = mcbComm.RX();
    }
}

void StratoPIB::HandleMCBASCII()
{
    switch (mcbComm.ascii_rx.msg_id) {
    case MCB_MOTION_FINISHED:
        log_nominal("MCB motion finished");
        mcb_motion = NO_MOTION;
        mcb_motion_ongoing = false;
        break;
    case MCB_ERROR:
        if (mcbComm.RX_Error(log_array, 101)) {
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    case MCB_MOTION_FAULT:
        if (mcbComm.RX_Motion_Fault(motion_fault, motion_fault+1, motion_fault+2, motion_fault+3,
                                    motion_fault+4, motion_fault+5, motion_fault+6, motion_fault+7)) {
            mcb_motion_ongoing = false;
            snprintf(log_array, 101, "MCB Fault: %u,%u,%u,%u,%u,%u,%u,%u", motion_fault[0], motion_fault[1],
                     motion_fault[2], motion_fault[3], motion_fault[4], motion_fault[5], motion_fault[6], motion_fault[7]);
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    default:
        log_error("Unknown MCB message received");
        break;
    }
}

void StratoPIB::HandleMCBAck()
{
    switch (mcbComm.ack_id) {
    case MCB_GO_LOW_POWER:
        log_nominal("MCB in low power");
        mcb_low_power = true;
        break;
    case MCB_REEL_IN:
        if (MOTION_REEL_IN == mcb_motion) mcb_motion_ongoing = true;
        break;
    case MCB_REEL_OUT:
        if (MOTION_REEL_OUT == mcb_motion) mcb_motion_ongoing = true;
        break;
    case MCB_DOCK:
        if (MOTION_DOCK == mcb_motion) mcb_motion_ongoing = true;
        break;
    case MCB_IN_ACC:
    case MCB_OUT_ACC:
    case MCB_DOCK_ACC:
        // currently not handled, though received
        break;
    default:
        log_error("Unknown MCB ack received");
        break;
    }
}

bool StratoPIB::StartMCBMotion()
{
    bool success = false;

    switch (mcb_motion) {
    case MOTION_REEL_IN:
        snprintf(log_array, 101, "Retracting %0.1f revs", retract_length);
        success = mcbComm.TX_Reel_In(retract_length, retract_velocity); // todo: verification
        break;
    case MOTION_REEL_OUT:
        snprintf(log_array, 101, "Deploying %0.1f revs", deploy_length);
        success = mcbComm.TX_Reel_Out(deploy_length, deploy_velocity); // todo: verification
        break;
    case MOTION_DOCK:
        snprintf(log_array, 101, "Docking %0.1f revs", dock_length);
        success = mcbComm.TX_Dock(dock_length, dock_velocity); // todo: verification
        break;
    case MOTION_UNDOCK:
    default:
        mcb_motion = NO_MOTION;
        log_error("Unknown motion type to start");
        return false;
    }

    if (autonomous_mode) {
        log_nominal(log_array);
    } else {
        ZephyrLogFine(log_array);
    }

    return success;
}