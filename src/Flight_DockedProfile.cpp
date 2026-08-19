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
    // shared standby+offload sequence: reached both at a periodic period
    // boundary and at the end of the profile. ST_OFFLOAD resumes measure
    // (loops back to ST_GO_MEASURE) unless resume_after_offload is false, which
    // is the only difference between a periodic pause and the final offload.
    ST_GO_STANDBY,
    ST_CONFIRM_STANDBY,
    ST_OFFLOAD,
};

static ProfileStates_t profile_state = ST_ENTRY;
static bool resend_attempted = false;
static uint32_t measure_remaining_s = 0;     // total measurement time still owed
static uint16_t period_len_s = 0;            // this period's commanded length
static uint32_t period_deadline_ms = 0;
static uint32_t pu_reply_deadline_ms = 0;
static bool resume_after_offload = false;    // true: go back to measure; false: profile is done

// Length of the next measure period. docked_offload_period == 0 means periodic
// offload is disabled (legacy behavior: one period covering the whole profile).
// A period shorter than the sample rate is folded into its neighbor: the RPU
// silently rewrites duration <= rate to rate + 1 (RPU.cpp), which would
// otherwise make RACHUTS and the RPU disagree about how long the period ran and
// break the "periods sum to docked_profile_time" invariant. docked_profile_time
// is guaranteed nonzero here -- a docked profile is never unbounded (ST_ENTRY
// rejects duration == 0), so remaining is always > 0 on entry and this never
// returns 0.
static uint16_t NextPeriodLength(uint16_t period, uint16_t rate, uint32_t remaining)
{
    if (period == 0) return (uint16_t) remaining;
    if (period <= rate) period = rate + 1;
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
    // standby+offload sequence used for a normal period boundary, just
    // without resuming measure afterward.
    if (CheckAction(ACTION_CANCEL_MEASURE)) {
        resume_after_offload = false;
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
            SendTextTM("Docked profile: invalid sample rate 0, returning to FLM_IDLE", WARN);
            return true;
        }
        // A docked profile must never be unbounded (see KnownIssues) -- the TC
        // parser already rejects duration == 0, this is defense-in-depth in case
        // docked_profile_time is ever set some other way.
        if (0 == docked_profile_time) {
            SendTextTM("Docked profile: invalid duration 0, returning to FLM_IDLE", WARN);
            return true;
        }
        // Reject up front if the longest period this profile will run would need
        // too many RPU record blocks (RPU_TM_MAX_RECORDS each): Flight_PUOffload
        // arms one RESEND_TM (and briefly one RESEND_PU_RECORD) per block without
        // cancelling the previous one, so a single offload needing too many
        // blocks alone can fill the 32-slot scheduler queue -- independent of how
        // well-spaced periodic offloads are from each other.
        {
            uint32_t longest_period_s = (0 == pibConfigs.docked_offload_period.Read())
                                       ? docked_profile_time
                                       : pibConfigs.docked_offload_period.Read();
            if (longest_period_s > (uint32_t) MAX_DOCKED_PERIOD_TMS * RPU_TM_MAX_RECORDS * docked_profile_rate) {
                SendTextTM("Docked profile: period too long for sample rate, returning to FLM_IDLE", WARN);
                return true;
            }
        }
        measure_remaining_s = docked_profile_time;
        resume_after_offload = false;
        resend_attempted = false;
        pibConfigs.profile_id.Write(pibConfigs.profile_id.Read() + 1);
        docked_period_num = 0;
        // Save the starting lat/lon/alt to include in the profile's RPUREPORTs
        // (mirrors PUStartProfile(), which the docked path doesn't call)
        profile_start_latitude = zephyrRX.zephyr_gps.latitude;
        profile_start_longitude = zephyrRX.zephyr_gps.longitude;
        profile_start_altitude = zephyrRX.zephyr_gps.altitude;
        profile_state = ST_GO_MEASURE;
        break;

    case ST_GO_MEASURE:
        // resend_attempted is NOT reset here: this state is re-entered both for a
        // fresh period (where it's already false, left that way by the last
        // successful confirm) and for a retry (where ST_CONFIRM_GO_MEASURE just
        // set it true) -- resetting it unconditionally would erase the retry
        // count and let the "give up after one retry" check below never fire.
        period_len_s = NextPeriodLength(pibConfigs.docked_offload_period.Read(),
                                        docked_profile_rate, measure_remaining_s);
        // Reset to a fresh false baseline before sending: pu_measure is set true
        // by PURouter (HandlePUAck) only when this go-measure command's own ack
        // arrives.
        pu_measure = false;
        pu_standby = false;
        puComm.TX_GoMeasure(period_len_s,
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
            // The RPU's own go-measure duration is also period_len_s, so it
            // should already be returning to standby on its own; give it a
            // couple seconds' head start before RACHUTS's own timer also sends
            // it to standby as a backup, so it's enforced in two places.
            period_deadline_ms = millis() + ((uint32_t) period_len_s * 1000UL) + 2000UL;
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
        if ((int32_t) (millis() - period_deadline_ms) < 0) break;
        measure_remaining_s -= (measure_remaining_s < period_len_s) ? measure_remaining_s : period_len_s;
        resume_after_offload = (measure_remaining_s > 0);
        profile_state = ST_GO_STANDBY;
        break;

    case ST_GO_STANDBY:
        // Same non-reset rule as ST_GO_MEASURE above, plus: only announce the
        // period on the fresh entry (resend_attempted == false), not on a
        // retry re-entry, so a slow/lost standby ack doesn't duplicate the
        // ground-facing text TM every RPU_RECEIVE_TIMEOUT seconds.
        if (!resend_attempted) {
            snprintf(log_array, LOG_ARRAY_SIZE, "Docked profile period complete%s",
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
            docked_period_num++;
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
        // ST_TM_ACK's uncapped TM resend looks unbounded, but it isn't: the
        // RESEND_TM scheduler timeout always eventually fires and forces
        // ST_TM_ACK onward regardless of ACK status, and pu_no_more_records
        // (checked independent of TM-ack history) terminates the offload once
        // the RPU's finite record supply runs out. Flight_PUOffload() always
        // eventually returns true.
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
