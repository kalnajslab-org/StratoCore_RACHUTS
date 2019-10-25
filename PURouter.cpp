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
        if (ASCII_MESSAGE == rx_msg) {
            HandlePUASCII();
        } else if (ACK_MESSAGE == rx_msg) {
            HandlePUAck();
        } else if (BIN_MESSAGE == rx_msg) {
            HandlePUBin();
        } else {
            log_error("Non-ASCII message from MCB");
        }

        rx_msg = puComm.RX();
    }
}

void StratoPIB::HandlePUASCII()
{
    switch (puComm.ascii_rx.msg_id) {
    case PU_STATUS:
        EEPROM_UPDATE_BOOL(pibStorage, pu_docked, true);
        if (!puComm.RX_Status(&PUTime, &PUVBattery, &PUICharge, &PUTherm1T, &PUTherm2T, &PUHeaterStat)) {
            PUTime = 0;
            PUVBattery = 0.0f;
            PUICharge = 0.0f;
            PUTherm1T = 0.0f;
            PUTherm2T = 0.0f;
            PUHeaterStat = 0;
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
    default:
        log_error("Unknown PU ack received");
        break;
    }
}

void StratoPIB::HandlePUBin()
{
    switch (puComm.binary_rx.bin_id) {
    // can handle all PU TM receipt here with ACKs/NAKs and tm_finished + buffer_ready flags
    case PU_TSEN_RECORD:
        // prep the TM buffer
        zephyrTX.clearTm();

        // see if we can place in the buffer
        if (zephyrTX.addTm(puComm.binary_rx.bin_buffer, puComm.binary_rx.bin_length)) {
            tsen_received = true;
            puComm.TX_Ack(PU_TSEN_RECORD, true);
        } else {
            puComm.TX_Ack(PU_TSEN_RECORD, false);
            zephyrTX.clearTm();
        }

        break;
    default:
        log_error("Unknown PU bin received");
    }
}