/*
 *  Flight_CheckPU.cpp
 *  Author:  Alex St. Clair
 *  Created: October 2019
 */

#include "StratoPIB.h"

enum InternalStates_t {
    ST_ENTRY,
    ST_SEND_REQUEST,
    ST_WAIT_REQUEST,
};

static InternalStates_t internal_state = ST_ENTRY;
static bool resend_attempted = false;
static uint32_t last_pu_status = 0;

bool StratoPIB::Flight_CheckPU(bool restart_state)
{
    if (restart_state) internal_state = ST_ENTRY;

    switch (internal_state) {
    case ST_ENTRY:
        log_nominal("Starting CheckPU Flight State");
        resend_attempted = false;
        last_pu_status = pu_status.last_status;
        internal_state = ST_SEND_REQUEST;
        break;

    case ST_SEND_REQUEST:
        puComm.TX_ASCII(PU_SEND_STATUS);
        scheduler.AddAction(RESEND_PU_CHECK, PU_RESEND_TIMEOUT);
        internal_state = ST_WAIT_REQUEST;
        break;

    case ST_WAIT_REQUEST:
        if (last_pu_status != pu_status.last_status) {
            resend_attempted = false;
            snprintf(log_array, LOG_ARRAY_SIZE, "PU status: %lu, %0.2f, %0.2f, %0.2f, %0.2f, %u", pu_status.time, pu_status.v_battery, pu_status.i_charge, pu_status.therm1, pu_status.therm2, pu_status.heater_stat);
            ZephyrLogFine(log_array);
            return true;
        }

        if (CheckAction(RESEND_PU_CHECK)) {
            if (!resend_attempted) {
                resend_attempted = true;
                internal_state = ST_SEND_REQUEST;
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