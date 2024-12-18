/*
 *  Flight_PUOffload.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoRatchuts.h"

enum PUOffloadStates_t {
    ST_ENTRY,
    ST_GET_PU_STATUS,
    ST_WAIT_PU_STATUS,
    ST_REQUEST_PACKET,
    ST_WAIT_PACKET,
    ST_TM_ACK,
};

static PUOffloadStates_t puoffload_state = ST_ENTRY;
static bool resend_attempted = false;
static uint8_t packet_num = 0;

bool StratoRatchuts::Flight_PUOffload(bool restart_state)
{
    if (restart_state) puoffload_state = ST_ENTRY;

    switch (puoffload_state) {
    case ST_ENTRY:
        resend_attempted = false;
        packet_num = 0;
        puoffload_state = ST_GET_PU_STATUS;
        break;

    case ST_GET_PU_STATUS:
        Flight_CheckPU(true);
        puoffload_state = ST_WAIT_PU_STATUS;
        break;

    case ST_WAIT_PU_STATUS:
        if (Flight_CheckPU(false)) {
            puoffload_state = ST_REQUEST_PACKET;
        }
        break;

    case ST_REQUEST_PACKET:
        puComm.TX_ASCII(PU_SEND_PROFILE_RECORD);
        scheduler.AddAction(RESEND_PU_RECORD, PU_RESEND_TIMEOUT);
        record_received = false;
        pu_no_more_records = false;
        puoffload_state = ST_WAIT_PACKET;
        break;

    case ST_WAIT_PACKET:
        if (record_received) { // ACK/NAK in PURouter
            record_received = false;
            packet_num++;
            snprintf(log_array, LOG_ARRAY_SIZE, "Received profile record: %u", puComm.binary_rx.bin_length);
            log_nominal(log_array);
            SendProfileTM(packet_num);
            puoffload_state = ST_TM_ACK;
            scheduler.AddAction(RESEND_TM, ZEPHYR_RESEND_TIMEOUT);
            break;
        } else if (pu_no_more_records) {
            pu_no_more_records = false;
            log_nominal("No more profile records");
            return true;
        }

        if (CheckAction(RESEND_PU_RECORD)) {
            if (!resend_attempted) {
                resend_attempted = true;
                puoffload_state = ST_REQUEST_PACKET;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not successful in sending profile record");
                return true;
            }
        }
        break;

    case ST_TM_ACK:
        if (ACK == TM_ack_flag) {
            resend_attempted = false;
            puoffload_state = ST_GET_PU_STATUS;
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // attempt one resend
            log_error("Needed to resend TM");
            zephyrTX.TM(); // message is still saved in XMLWriter, no need to reconstruct
            resend_attempted = false;
            puoffload_state = ST_GET_PU_STATUS;
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}