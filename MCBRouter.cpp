/*
 *  MCBRouter.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Motor Control Board message router and handlers.
 */

#include "StratoPIB.h"
#include "Serialize.h"

void StratoPIB::RunMCBRouter()
{
    SerialMessage_t rx_msg = mcbComm.RX();

    while (NO_MESSAGE != rx_msg) {
        if (ASCII_MESSAGE == rx_msg) {
            HandleMCBASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandleMCBAck();
        } else if (BIN_MESSAGE == rx_msg) {
            HandleMCBBin();
        } else {
            log_error("Unknown message type from MCB");
        }

        rx_msg = mcbComm.RX();
    }
}

void StratoPIB::HandleMCBASCII()
{
    switch (mcbComm.ascii_rx.msg_id) {
    case MCB_MOTION_FINISHED:
        log_nominal("MCB motion finished"); // state machine will report to Zephyr
        mcb_motion_ongoing = false;
        break;
    case MCB_ERROR:
        if (mcbComm.RX_Error(log_array, LOG_ARRAY_SIZE)) {
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    case MCB_MOTION_FAULT:
        // expected if docking
        if (mcb_dock_ongoing) { // todo: ensure the correct motion fault flags for dock
            log_nominal("Dock condition detected");
            mcb_dock_ongoing = false;
            mcb_motion_ongoing = false;
            inst_substate = MODE_ENTRY; // re-enter the mode
            break;
        }

        if (mcbComm.RX_Motion_Fault(motion_fault, motion_fault+1, motion_fault+2, motion_fault+3,
                                    motion_fault+4, motion_fault+5, motion_fault+6, motion_fault+7)) {
            mcb_motion_ongoing = false;
            snprintf(log_array, LOG_ARRAY_SIZE, "MCB Fault: %x,%x,%x,%x,%x,%x,%x,%x", motion_fault[0], motion_fault[1],
                     motion_fault[2], motion_fault[3], motion_fault[4], motion_fault[5], motion_fault[6], motion_fault[7]);
            SendMCBTM(CRIT, log_array);
            inst_substate = MODE_ERROR;
        } else {
            mcb_motion_ongoing = false;
            SendMCBTM(CRIT, "MCB Fault: error receiving parameters");
            inst_substate = MODE_ERROR;
        }
        break;
    default:
        log_error("Unknown MCB ASCII message received");
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
        if (MOTION_REEL_IN == mcb_motion) NoteProfileStart();
        break;
    case MCB_REEL_OUT:
        if (MOTION_REEL_OUT == mcb_motion) NoteProfileStart();
        break;
    case MCB_DOCK:
        if (MOTION_DOCK == mcb_motion) NoteProfileStart();
        break;
    case MCB_IN_NO_LW:
        if (MOTION_IN_NO_LW == mcb_motion) NoteProfileStart();
        break;
    case MCB_IN_ACC:
    case MCB_OUT_ACC:
    case MCB_DOCK_ACC:
    case MCB_ZERO_REEL:
        // currently not handled, though received
        break;
    default:
        log_error("Unknown MCB ack received");
        break;
    }
}

void StratoPIB::HandleMCBBin()
{
    float reel_pos = 0;
    uint16_t reel_pos_index = 21; // todo: don't hard-code this

    switch (mcbComm.binary_rx.bin_id) {
    case MCB_MOTION_TM:
        if (BufferGetFloat(&reel_pos, mcbComm.binary_rx.bin_buffer, mcbComm.binary_rx.bin_length, &reel_pos_index)) {
            snprintf(log_array, 101, "Reel position: %ld", (int32_t) reel_pos);
            log_nominal(log_array);
        } else {
            log_nominal("Recieved MCB bin: unable to read position");
        }
        AddMCBTM();
        break;
    default:
        log_error("Unknown MCB bin received");
    }
}
