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