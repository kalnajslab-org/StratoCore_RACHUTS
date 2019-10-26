/*
 *  Flight_PUOffload.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoPIB.h"

enum InternalStates_t {
    ST_ENTRY,
    ST_REQUEST_PACKET,
    ST_WAIT_PACKET,
    ST_TM_ACK,
};

static InternalStates_t internal_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoPIB::Flight_PUOffload(bool restart_state)
{
    if (restart_state) internal_state = ST_ENTRY;

    switch (internal_state) {
    case ST_ENTRY:
        resend_attempted = false;
        internal_state = ST_REQUEST_PACKET;
        break;

    case ST_REQUEST_PACKET:
        puComm.TX_ASCII(PU_SEND_PROFILE_RECORD);
        scheduler.AddAction(RESEND_PU_RECORD, PU_RESEND_TIMEOUT);
        record_received = false;
        pu_no_more_records = false;
        internal_state = ST_WAIT_PACKET;
        break;

    case ST_WAIT_PACKET:
        if (record_received) { // ACK/NAK in PURouter
            record_received = false;
            log_nominal("Received profile record");
            SendProfileTM();
            internal_state = ST_TM_ACK;
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
                internal_state = ST_REQUEST_PACKET;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not successful in sending profile record");
                return true;
            }
        }

    case ST_TM_ACK:
        if (ACK == TM_ack_flag) {
            internal_state = ST_ENTRY;
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // attempt one resend
            log_error("Needed to resend TM");
            zephyrTX.TM(); // message is still saved in XMLWriter, no need to reconstruct
            internal_state = ST_ENTRY;
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}