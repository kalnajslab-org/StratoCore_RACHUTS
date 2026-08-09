/*
 *  PURouter.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Profiling Unit message router and handlers.
 */

#include "StratoRachuts.h"

void StratoRachuts::RunPURouter()
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

void StratoRachuts::HandlePUASCII()
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

void StratoRachuts::HandlePUAck()
{
    switch (puComm.ack_id) {
    case RPU_GO_MEASURE:
        if (puComm.ack_value) {
            log_nominal("RPU in measure");
            pu_measure = true;
        } else {
            SendTextTM("RPU NAKed go-measure command", WARN);
        }
        break;
    case RPU_GO_STANDBY:
        if (puComm.ack_value) {
            log_nominal("RPU in standby");
        } else {
            SendTextTM("RPU NAKed go-standby command", WARN);
        }
        break;
    case RPU_RESET:
        SendTextTM("RPU acked reset", FINE);
        break;
    case RPU_SET_STATUS_RATE:
        log_nominal("RPU acked status rate");
        break;
    default:
        log_error("Unknown RPU ack received");
        break;
    }
}

void StratoRachuts::HandlePUBin()
{
    // can handle all PU TM receipt here with ACKs/NAKs and tm_finished + buffer_ready flags
    switch (puComm.binary_rx.bin_id) {
    case RPU_PROFILE_RECORD:
        if (!puComm.binary_rx.checksum_valid) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Profile record checksum invalid (len=%u)",
                     puComm.binary_rx.bin_length);
            log_error(log_array);
            puComm.TX_Ack(RPU_PROFILE_RECORD, false);
            zephyrTX.clearTm();
        } else if (!zephyrTX.addTm(puComm.binary_rx.bin_buffer, puComm.binary_rx.bin_length)) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Profile record too large for TM buffer (len=%u, tm_used=%u)",
                     puComm.binary_rx.bin_length, zephyrTX.getTmLen());
            log_error(log_array);
            puComm.TX_Ack(RPU_PROFILE_RECORD, false);
            zephyrTX.clearTm();
        } else {
            record_received = true;
            puComm.TX_Ack(RPU_PROFILE_RECORD, true);
        }
        break;

    case RPU_STATUS: {
        char json_buf[512];
        if (puComm.binary_rx.checksum_valid && puComm.RX_Status(json_buf, sizeof(json_buf))) {
            pu_status_json = json_buf;
            pu_last_status = now();
            pu_status_received = true;
            latest_rpu_json = json_buf;
            latest_rpu_src = "DOCK";
            last_rpu_recv_ms = millis();
            rpu_ever_received = true;
            rpu_status_pending = true;
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

void StratoRachuts::HandlePUString()
{
    switch (puComm.string_rx.str_id) {
    case RPU_ERROR:
        if (puComm.RX_Error(log_array, LOG_ARRAY_SIZE)) {
            SendTextTM(log_array, CRIT);
            inst_substate = MODE_ERROR;
        }
        break;
    default:
        log_error("Unknown PU String message received");
        break;
    }
}