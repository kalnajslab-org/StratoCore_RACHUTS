/*
 *  Flight_CheckPU.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoRatchuts.h"

enum CheckPUStates_t {
    ST_ENTRY,
    ST_SEND_REQUEST,
    ST_WAIT_REQUEST,
};

static CheckPUStates_t checkpu_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoRatchuts::Flight_CheckPU(bool restart_state)
{
    if (restart_state) checkpu_state = ST_ENTRY;

    switch (checkpu_state) {
    case ST_ENTRY:
        log_nominal("Starting CheckPU Flight State");
        resend_attempted = false;
        check_pu_success = false;
        pu_status_received = false;
        checkpu_state = ST_SEND_REQUEST;
        break;

    case ST_SEND_REQUEST:
        puComm.TX_ASCII(RPU_SEND_STATUS);
        scheduler.AddAction(RESEND_PU_CHECK, PU_RESEND_TIMEOUT);
        if (resend_attempted) {
            log_nominal("CheckPU: resent RPU_SEND_STATUS request");
        } else {
            log_nominal("CheckPU: sent RPU_SEND_STATUS request");
        }
        checkpu_state = ST_WAIT_REQUEST;
        break;

    case ST_WAIT_REQUEST:
        if (pu_status_received) {
            log_nominal("CheckPU: status received");
            resend_attempted = false;
            check_pu_success = true;
            return true;
        }

        if (CheckAction(RESEND_PU_CHECK)) {
            if (!resend_attempted) {
                resend_attempted = true;
                log_nominal("CheckPU: resend timeout, retrying request");
                checkpu_state = ST_SEND_REQUEST;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not responding to status request");
                return true;
            }
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}