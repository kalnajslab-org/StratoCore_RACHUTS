/*
 *  Flight_DockedProfile.cpp
 *  Author:  Alex St. Clair
 *  Created: June 2020
 */

#include "StratoRatchuts.h"

enum ProfileStates_t {
    ST_ENTRY,
    ST_GO_MEASURE,
    ST_CONFIRM_GO_MEASURE,
    ST_MEASURE_WAIT,
};

static ProfileStates_t profile_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoRatchuts::Flight_DockedProfile(bool restart_state)
{
    if (restart_state) profile_state = ST_ENTRY;

    switch (profile_state) {
    case ST_ENTRY:
    case ST_GO_MEASURE:
        pu_measure = false;
        resend_attempted = false;
        puComm.TX_GoMeasure(pibConfigs.rpu_meas_duration.Read(),
                            pibConfigs.rpu_meas_rate.Read(),
                            pibConfigs.rpu_bat_temp.Read(),
                            pibConfigs.rpu_enable_ROPC.Read(),
                            pibConfigs.rpu_enable_TDLAS.Read(),
                            pibConfigs.rpu_enable_TSEN.Read(),
                            pibConfigs.rpu_enable_RS41.Read());
        scheduler.AddAction(RESEND_PU_GOPROFILE, PU_RESEND_TIMEOUT);
        profile_state = ST_CONFIRM_GO_MEASURE;
        break;

    case ST_CONFIRM_GO_MEASURE:
        if (pu_measure) {
            pibConfigs.profile_id.Write(pibConfigs.profile_id.Read() + 1);
            scheduler.AddAction(ACTION_END_PREPROFILE, docked_profile_time);
            profile_state = ST_MEASURE_WAIT;
        } else if (CheckAction(RESEND_PU_GOPROFILE)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_GO_MEASURE;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("RPU not responding to go-measure command");
                return true;
            }
        }
        break;

    case ST_MEASURE_WAIT:
        if (CheckAction(ACTION_END_PREPROFILE)) {
            ZephyrLogFine("Finished docked profile");
            if (pibConfigs.pu_auto_offload.Read()) {
                Serial.println("Begin Automatic PU Offload");
                SetAction(ACTION_OFFLOAD_PU);
            }
            return true;
        }
        break;

    default:
        return true;
    }

    return false;
}
