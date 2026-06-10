/*
 *  PURouter.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Profiling Unit message router and handlers.
 */

#include "StratoRatchuts.h"

void StratoRatchuts::RunPURouter()
{
    SerialMessage_t rx_msg = puComm.RX();

    while (NO_MESSAGE != rx_msg) {
        PUDock();
        if (ASCII_MESSAGE == rx_msg) {
            HandlePUASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandlePUAck();
        } else if (BIN_MESSAGE == rx_msg) {
            HandlePUBin();
        } else if (STRING_MESSAGE == rx_msg) {
            HandlePUString();
        } else {
            log_error("Unknown message type from PU");
        }

        rx_msg = puComm.RX();
    }
}

void StratoRatchuts::HandlePUASCII()
{
    switch (puComm.ascii_rx.msg_id) {
    case RPU_NO_MORE_RECORDS:
        pu_no_more_records = true;
        break;
    default:
        log_error("Unknown PU ASCII message received");
        break;
    }
}

void StratoRatchuts::HandlePUAck()
{
    switch (puComm.ack_id) {
    case RPU_GO_MEASURE:
        log_nominal("RPU in measure");
        pu_measure = true;
        break;
    case RPU_GO_STANDBY:
        log_nominal("RPU in standby");
        break;
    case RPU_RESET:
        ZephyrLogFine("RPU acked reset");
        break;
    case RPU_SET_STATUS_RATE:
        log_nominal("RPU acked status rate");
        break;
    default:
        log_error("Unknown RPU ack received");
        break;
    }
}

void StratoRatchuts::HandlePUBin()
{
    // can handle all PU TM receipt here with ACKs/NAKs and tm_finished + buffer_ready flags
    switch (puComm.binary_rx.bin_id) {
    case RPU_PROFILE_RECORD:
        if (puComm.binary_rx.checksum_valid && zephyrTX.addTm(puComm.binary_rx.bin_buffer, puComm.binary_rx.bin_length)) {
            record_received = true;
            puComm.TX_Ack(RPU_PROFILE_RECORD, true);
        } else {
            log_error("Profile record checksum invalid or error adding to TM buffer");
            puComm.TX_Ack(RPU_PROFILE_RECORD, false);
            zephyrTX.clearTm();
        }
        break;

    case RPU_STATUS: {
        char json_buf[512];
        if (puComm.binary_rx.checksum_valid && puComm.RX_Status(json_buf, sizeof(json_buf))) {
            pu_status_json = json_buf;
            pu_last_status = now();
        } else {
            pu_status_json = "";
        }
        break;
    }

    default:
        log_error("Unknown PU bin received");
        break;
    }
}

void StratoRatchuts::HandlePUString()
{
    switch (puComm.string_rx.str_id) {
    case RPU_ERROR:
        if (puComm.RX_Error(log_array, LOG_ARRAY_SIZE)) {
            ZephyrLogCrit(log_array);
            inst_substate = MODE_ERROR;
        }
        break;
    default:
        log_error("Unknown PU String message received");
        break;
    }
}