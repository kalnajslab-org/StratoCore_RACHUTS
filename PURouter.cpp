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
        pu_docked = true; // todo: read status info
        if (!puComm.RX_Status(&PUTime, &PUVBattery, &PUICharge, &PUTherm1T, &PUTherm2T, &PUHeaterStat)) {
            PUTime = 0;
            PUVBattery = 0.0f;
            PUICharge = 0.0f;
            PUTherm1T = 0.0f;
            PUTherm2T = 0.0f;
            PUHeaterStat = 0;
        }
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
    default:
        log_error("Unknown PU bin received");
    }
}