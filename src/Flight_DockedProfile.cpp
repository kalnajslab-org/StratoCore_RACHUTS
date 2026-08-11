/*
 *  Flight_DockedProfile.cpp
 *  Author:  Alex St. Clair
 *  Created: June 2020
 */

#include "StratoRachuts.h"

enum ProfileStates_t {
    ST_ENTRY,
    ST_GO_MEASURE,
    ST_CONFIRM_GO_MEASURE,
    ST_MEASURE_WAIT,
    // shared standby+offload sequence: reached both at a periodic segment
    // boundary and at the end of the profile. ST_OFFLOAD resumes measure
    // (loops back to ST_GO_MEASURE) unless resume_after_offload is false, which
    // is the only difference between a periodic pause and the final offload.
    ST_GO_STANDBY,
    ST_CONFIRM_STANDBY,
    ST_OFFLOAD,
};

static ProfileStates_t profile_state = ST_ENTRY;
static bool resend_attempted = false;
static bool indefinite = false;             // docked_profile_time == 0: run until cancelled
static uint32_t measure_remaining_s = 0;     // total measurement time still owed
static uint16_t segment_len_s = 0;           // this segment's commanded length (0 = unlimited)
static uint32_t segment_deadline_ms = 0;
static uint32_t pu_reply_deadline_ms = 0;
static bool resume_after_offload = false;    // true: go back to measure; false: profile is done

// Length of the next measure segment. period == 0 means periodic offload is
// disabled (legacy behavior: one segment covering the whole profile, or
// unlimited if indefinite). A segment shorter than the sample rate is folded
// into its neighbor: the RPU silently rewrites duration <= rate to rate + 1
// (RPU.cpp), which would otherwise make RACHUTS and the RPU disagree about how
// long the segment ran and break the "segments sum to docked_profile_time"
// invariant.
static uint16_t NextSegmentLength(uint16_t period, uint16_t rate, uint32_t remaining, bool indef)
{
    if (period == 0) return indef ? 0 : (uint16_t) remaining;
    if (period <= rate) period = rate + 1;
    if (indef) return period;
    if (remaining <= period) return (uint16_t) remaining;
    if ((remaining - period) <= rate) return (uint16_t) remaining; // fold a sub-sample tail in
    return period;
}

bool StratoRachuts::Flight_DockedProfile(bool restart_state)
{
    if (restart_state) profile_state = ST_ENTRY;

    // Checked every state (not just one, per the KnownIssues §14 pitfall) so a
    // cancel takes effect regardless of which phase the profile is in. The RPU
    // is already commanded to standby unconditionally in TCHandler; this just
    // decides what the state machine does about it. If an offload is already
    // in progress, let it finish and come down to the ground rather than
    // abandoning it or restarting a second one; otherwise route into the same
    // standby+offload sequence used for a normal segment boundary, just
    // without resuming measure afterward.
    if (CheckAction(ACTION_CANCEL_MEASURE)) {
        resume_after_offload = false;
        indefinite = false;
        measure_remaining_s = 0;
        if (ST_OFFLOAD == profile_state) {
            log_nominal("Cancel received during offload; finishing current offload");
        } else if (ST_GO_STANDBY != profile_state && ST_CONFIRM_STANDBY != profile_state) {
            SendTextTM("Docked profile cancelled", FINE);
            resend_attempted = false; // fresh entry into the standby/offload sequence
            profile_state = ST_GO_STANDBY;
        }
        return false; // stay in FLM_DOCKED until the data is down
    }

    switch (profile_state) {
    case ST_ENTRY:
        if (0 == docked_profile_rate) {
            SendTextTM("Docked profile: invalid rate, returning to FLM_IDLE", WARN);
            return true;
        }
        indefinite = (0 == docked_profile_time);
        measure_remaining_s = docked_profile_time;
        resume_after_offload = false;
        resend_attempted = false;
        pibConfigs.profile_id.Write(pibConfigs.profile_id.Read() + 1);
        docked_segment = 0;
        // Save the starting lat/lon/alt to include in the profile's RPUREPORTs
        // (mirrors PUStartProfile(), which the docked path doesn't call)
        profile_start_latitude = zephyrRX.zephyr_gps.latitude;
        profile_start_longitude = zephyrRX.zephyr_gps.longitude;
        profile_start_altitude = zephyrRX.zephyr_gps.altitude;
        profile_state = ST_GO_MEASURE;
        break;

    case ST_GO_MEASURE:
        // resend_attempted is NOT reset here: this state is re-entered both for a
        // fresh segment (where it's already false, left that way by the last
        // successful confirm) and for a retry (where ST_CONFIRM_GO_MEASURE just
        // set it true) -- resetting it unconditionally would erase the retry
        // count and let the "give up after one retry" check below never fire.
        segment_len_s = NextSegmentLength(pibConfigs.docked_offload_period.Read(),
                                          docked_profile_rate, measure_remaining_s, indefinite);
        pu_measure = false;
        pu_standby = false;
        puComm.TX_GoMeasure(segment_len_s,
                            docked_profile_rate,
                            pibConfigs.rpu_bat_temp.Read(),
                            pibConfigs.rpu_enable_ROPC.Read(),
                            pibConfigs.rpu_enable_TDLAS.Read(),
                            pibConfigs.rpu_enable_TSEN.Read(),
                            pibConfigs.rpu_enable_RS41.Read());
        pu_reply_deadline_ms = millis() + (RPU_RECEIVE_TIMEOUT * 1000UL);
        profile_state = ST_CONFIRM_GO_MEASURE;
        break;

    case ST_CONFIRM_GO_MEASURE:
        if (pu_measure) {
            resend_attempted = false;
            // The RPU's own go-measure duration is also segment_len_s, so it
            // should already be returning to standby on its own; give it a
            // couple seconds' head start before RACHUTS's own timer also sends
            // it to standby as a backup, so it's enforced in two places.
            segment_deadline_ms = millis() + ((uint32_t) segment_len_s * 1000UL) + 2000UL;
            profile_state = ST_MEASURE_WAIT;
        } else if ((int32_t) (millis() - pu_reply_deadline_ms) >= 0) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_GO_MEASURE;
            } else {
                resend_attempted = false;
                SendTextTM("RPU not responding to go-measure command, returning to FLM_IDLE", WARN);
                return true;
            }
        }
        break;

    case ST_MEASURE_WAIT:
        if (0 == segment_len_s) break; // indefinite, no periodic offload: wait for cancel only
        if ((int32_t) (millis() - segment_deadline_ms) < 0) break;
        if (!indefinite) {
            measure_remaining_s -= (measure_remaining_s < segment_len_s) ? measure_remaining_s : segment_len_s;
        }
        resume_after_offload = (indefinite || measure_remaining_s > 0);
        profile_state = ST_GO_STANDBY;
        break;

    case ST_GO_STANDBY:
        // Same non-reset rule as ST_GO_MEASURE above, plus: only announce the
        // segment on the fresh entry (resend_attempted == false), not on a
        // retry re-entry, so a slow/lost standby ack doesn't duplicate the
        // ground-facing text TM every RPU_RECEIVE_TIMEOUT seconds.
        if (!resend_attempted) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Docked profile segment complete%s",
                    resume_after_offload ? "; offloading and resuming measure" : "; offloading, finished");
            SendTextTM(log_array, FINE);
        }
        pu_standby = false;
        puComm.TX_GoStandby(pibConfigs.rpu_bat_temp.Read());
        pu_reply_deadline_ms = millis() + (RPU_RECEIVE_TIMEOUT * 1000UL);
        profile_state = ST_CONFIRM_STANDBY;
        break;

    case ST_CONFIRM_STANDBY:
        if (pu_standby) {
            resend_attempted = false;
            docked_segment++;
            Flight_PUOffload(true);
            profile_state = ST_OFFLOAD;
        } else if ((int32_t) (millis() - pu_reply_deadline_ms) >= 0) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_GO_STANDBY;
            } else {
                // Two unacknowledged go-standby commands means the dock link is
                // down, so the offload cannot succeed either. Attempting it
                // anyway just burns ~40 s of CheckPU and record-request timeouts
                // to reach the same conclusion, emitting a cascade of WARN TMs
                // on the way (observed on the bench). Abort here instead.
                resend_attempted = false;
                SendTextTM("RPU did not confirm standby; aborting docked profile, returning to FLM_IDLE", WARN);
                return true;
            }
        }
        break;

    case ST_OFFLOAD:
        if (!Flight_PUOffload(false)) break;
        if (!pu_offload_success) {
            SendTextTM("Docked profile aborted: RPU offload failed, returning to FLM_IDLE", WARN);
            return true;
        }
        if (!resume_after_offload) {
            SendTextTM("Finished docked profile, returning to FLM_IDLE", FINE);
            return true;
        }
        profile_state = ST_GO_MEASURE;
        break;

    default:
        return true;
    }

    return false;
}
