/*
 *  Flight_TSEN.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoPIB.h"

enum InternalStates_t {
    ST_ENTRY,
    ST_REQUEST_TSEN,
    ST_WAIT_TSEN,
    ST_TM_ACK,
};

static InternalStates_t internal_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoPIB::Flight_TSEN(bool restart_state)
{
    if (restart_state) internal_state = ST_ENTRY;

    switch (internal_state) {
    case ST_ENTRY:
        resend_attempted = false;
        internal_state = ST_REQUEST_TSEN;
        break;

    case ST_REQUEST_TSEN:
        puComm.TX_ASCII(PU_SEND_TSEN_RECORD);
        scheduler.AddAction(RESEND_PU_TSEN, PU_RESEND_TIMEOUT);
        tsen_received = false;
        pu_no_more_records = false;
        internal_state = ST_WAIT_TSEN;
        break;

    case ST_WAIT_TSEN:
        if (tsen_received) { // ACK/NAK in PURouter
            tsen_received = false;
            log_nominal("Received TSEN");
            SendTSENTM();
            internal_state = ST_TM_ACK;
            scheduler.AddAction(RESEND_TM, ZEPHYR_RESEND_TIMEOUT);
            break;
        } else if (pu_no_more_records) {
            pu_no_more_records = false;
            log_nominal("No more TSEN records");
            return true;
        }

        if (CheckAction(RESEND_PU_TSEN)) {
            if (!resend_attempted) {
                resend_attempted = true;
                internal_state = ST_REQUEST_TSEN;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not successful in sending TSEN");
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