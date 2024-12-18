/*
 *  Flight_TSEN.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoRatchuts.h"

enum TSENStates_t {
    ST_ENTRY,
    ST_GET_PU_STATUS,
    ST_REQUEST_TSEN,
    ST_WAIT_TSEN,
    ST_TM_ACK,
};

static TSENStates_t tsen_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoRatchuts::Flight_TSEN(bool restart_state)
{
    // TSEN is overrideable in manual mode if a command is received, or autonomous if it's profile time
    if (!autonomous_mode && CheckAction(ACTION_OVERRIDE_TSEN)) {
        return true; // kill the TSEN state
    } else if (autonomous_mode && CheckAction(ACTION_BEGIN_PROFILE)) {
        SetAction(ACTION_BEGIN_PROFILE);
        return true;
    }

    if (restart_state) tsen_state = ST_ENTRY;

    switch (tsen_state) {
    case ST_ENTRY:
        resend_attempted = false;
        Flight_CheckPU(true);
        tsen_state = ST_GET_PU_STATUS;
        break;

    case ST_GET_PU_STATUS:
        if (Flight_CheckPU(false)) {
            tsen_state = ST_REQUEST_TSEN;
        }
        break;

    case ST_REQUEST_TSEN:
        puComm.TX_ASCII(PU_SEND_TSEN_RECORD);
        scheduler.AddAction(RESEND_PU_TSEN, PU_RESEND_TIMEOUT);
        tsen_received = false;
        pu_no_more_records = false;
        tsen_state = ST_WAIT_TSEN;
        break;

    case ST_WAIT_TSEN:
        if (tsen_received) { // ACK/NAK in PURouter
            tsen_received = false;
            snprintf(log_array, LOG_ARRAY_SIZE, "Received TSEN: %u", puComm.binary_rx.bin_length);
            log_nominal(log_array);
            SendTSENTM();
            tsen_state = ST_TM_ACK;
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
                tsen_state = ST_REQUEST_TSEN;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not successful in sending TSEN");
                return true;
            }
        }
        break;

    case ST_TM_ACK:
        if (ACK == TM_ack_flag) {
            tsen_state = ST_ENTRY;
        } else if (NAK == TM_ack_flag || CheckAction(RESEND_TM)) {
            // attempt one resend
            log_error("Needed to resend TM");
            zephyrTX.TM(); // message is still saved in XMLWriter, no need to reconstruct
            tsen_state = ST_ENTRY;
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}