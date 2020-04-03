/*
 *  PURouter.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 *
 *  This file implements the RACHuTS Profiling Unit message router and handlers.
 */

#include "StratoPIB.h"

void StratoPIB::RunPURouter()
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

void StratoPIB::HandlePUASCII()
{
    switch (puComm.ascii_rx.msg_id) {
    case PU_STATUS:
        if (!puComm.ascii_rx.checksum_valid || !puComm.RX_Status(&pu_status.time, &pu_status.v_battery, &pu_status.i_charge, &pu_status.therm1, &pu_status.therm2, &pu_status.heater_stat)) {
            pu_status.time = 0;
            pu_status.v_battery = 0.0f;
            pu_status.i_charge = 0.0f;
            pu_status.therm1 = 0.0f;
            pu_status.therm2 = 0.0f;
            pu_status.heater_stat = 0;
        } else {
            pu_status.last_status = now();
        }
        break;
    case PU_NO_MORE_RECORDS:
        pu_no_more_records = true;
        break;
    default:
        log_error("Unknown PU ASCII message received");
        break;
    }
}

void StratoPIB::HandlePUAck()
{
    switch (puComm.ack_id) {
    case PU_GO_WARMUP:
        log_nominal("PU in warmup");
        pu_warmup = true;
        break;
    case PU_GO_PROFILE:
        log_nominal("PU in profile");
        pu_profile = true;
        break;
    case PU_RESET:
        ZephyrLogFine("PU acked reset");
        break;
    default:
        log_error("Unknown PU ack received");
        break;
    }
}

void StratoPIB::HandlePUBin()
{
    // can handle all PU TM receipt here with ACKs/NAKs and tm_finished + buffer_ready flags
    switch (puComm.binary_rx.bin_id) {
    case PU_TSEN_RECORD:
        // prep the TM buffer
        zephyrTX.clearTm();

        // see if we can place in the buffer
        if (puComm.binary_rx.checksum_valid && zephyrTX.addTm(puComm.binary_rx.bin_buffer, puComm.binary_rx.bin_length)) {
            tsen_received = true;
            puComm.TX_Ack(PU_TSEN_RECORD, true);
        } else {
            log_error("TSEN checksum invalid or error adding to TM buffer");
            puComm.TX_Ack(PU_TSEN_RECORD, false);
            zephyrTX.clearTm();
        }
        break;

    case PU_PROFILE_RECORD:
        // prep the TM buffer
        zephyrTX.clearTm();

        // see if we can place in the buffer
        if (puComm.binary_rx.checksum_valid && zephyrTX.addTm(puComm.binary_rx.bin_buffer, puComm.binary_rx.bin_length)) {
            record_received = true;
            puComm.TX_Ack(PU_TSEN_RECORD, true);
        } else {
            log_error("Profile record checksum invalid or error adding to TM buffer");
            puComm.TX_Ack(PU_TSEN_RECORD, false);
            zephyrTX.clearTm();
        }
        break;

    default:
        log_error("Unknown PU bin received");
        break;
    }
}

void StratoPIB::HandlePUString()
{
    switch (puComm.string_rx.str_id) {
    case PU_ERROR:
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