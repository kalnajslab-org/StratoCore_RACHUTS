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

    // Checked every state (not just one, per the KnownIssues §14 pitfall) so a
    // cancel takes effect regardless of which phase the profile is in. The RPU
    // is already commanded to standby unconditionally in TCHandler; this just
    // stops the state machine from continuing to wait on it.
    if (CheckAction(ACTION_CANCEL_MEASURE)) {
        SendTextTM("Docked profile cancelled", FINE);
        SetAction(ACTION_OFFLOAD_PU);
        return true;
    }

    switch (profile_state) {
    case ST_ENTRY:
    case ST_GO_MEASURE:
        pu_measure = false;
        resend_attempted = false;
        puComm.TX_GoMeasure(docked_profile_time,
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
            // The RPU's go-measure duration is also docked_profile_time, so it
            // should already be returning to standby on its own; give it a
            // couple seconds' head start before RACHUTS's own timer also sends
            // it to standby as a backup, so it's enforced in two places.
            scheduler.AddAction(ACTION_END_PREPROFILE, docked_profile_time + 2);
            profile_state = ST_MEASURE_WAIT;
        } else if (CheckAction(RESEND_PU_GOPROFILE)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_GO_MEASURE;
            } else {
                resend_attempted = false;
                SendTextTM("RPU not responding to go-measure command", WARN);
                return true;
            }
        }
        break;

    case ST_MEASURE_WAIT:
        if (CheckAction(ACTION_END_PREPROFILE)) {
            SendTextTM("Finished docked profile", FINE);
            puComm.TX_GoStandby(pibConfigs.rpu_bat_temp.Read());
            SetAction(ACTION_OFFLOAD_PU);
            return true;
        }
        break;

    default:
        return true;
    }

    return false;
}
