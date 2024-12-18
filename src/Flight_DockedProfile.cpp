/*
 *  Flight_DockedProfile.cpp
 *  Author:  Alex St. Clair
 *  Created: June 2020
 */

#include "StratoRatchuts.h"

enum ProfileStates_t {
    ST_ENTRY,
    ST_SET_PU_WARMUP,
    ST_CONFIRM_PU_WARMUP,
    ST_WARMUP,
    ST_GET_TSEN,
    ST_SET_PU_PREPROFILE,
    ST_CONFIRM_PU_PREPROFILE,
    ST_PREPROFILE_WAIT,
    ST_GET_PU_STATUS,
};

static ProfileStates_t profile_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoRatchuts::Flight_DockedProfile(bool restart_state)
{
    if (restart_state) profile_state = ST_ENTRY;

    switch (profile_state) {
    case ST_ENTRY:
    case ST_SET_PU_WARMUP:
        pu_warmup = false;
        puComm.TX_WarmUp(pibConfigs.flash_temp.Read(),pibConfigs.heater1_temp.Read(),pibConfigs.heater2_temp.Read(),
                         pibConfigs.flash_power.Read(),pibConfigs.tsen_power.Read());
        scheduler.AddAction(RESEND_PU_WARMUP, PU_RESEND_TIMEOUT);
        profile_state = ST_CONFIRM_PU_WARMUP;
        break;

    case ST_CONFIRM_PU_WARMUP:
        if (pu_warmup) {
            profile_state = ST_WARMUP;
            scheduler.AddAction(ACTION_END_WARMUP, pibConfigs.puwarmup_time.Read());
        } else if (CheckAction(RESEND_PU_WARMUP)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_SET_PU_WARMUP;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not responding to warmup command");
                return true;
            }
        }
        break;

    case ST_WARMUP:
        if (CheckAction(ACTION_END_WARMUP)) {
            Flight_TSEN(true);
            profile_state = ST_GET_TSEN;
        }
        break;

    case ST_GET_TSEN:
        if (Flight_TSEN(false)) {
            profile_state = ST_SET_PU_PREPROFILE;
        }
        break;

    case ST_SET_PU_PREPROFILE:
        pu_preprofile = false;

        // FIXME: replace the TX_PreProfile with a dedicated command, figure out data transmission
        //LEK 8_2021: Just use the profile command to PU with short dwell and up times and LoRa off
        pu_preprofile = puComm.TX_Profile(docked_profile_time-10, 5,5, pibConfigs.docked_rate.Read(), 1,pibConfigs.docked_TSEN.Read(),
                             pibConfigs.docked_ROPC.Read(), pibConfigs.docked_FLASH.Read(),0);

        scheduler.AddAction(RESEND_PU_GOPROFILE, PU_RESEND_TIMEOUT);
        profile_state = ST_CONFIRM_PU_PREPROFILE;
        break;

    case ST_CONFIRM_PU_PREPROFILE:
        if (pu_preprofile) {
            profile_state = ST_PREPROFILE_WAIT;
            scheduler.AddAction(ACTION_END_PREPROFILE, docked_profile_time);
            pibConfigs.profile_id.Write(pibConfigs.profile_id.Read()+1); //increment the profile counter
        } else if (CheckAction(RESEND_PU_GOPROFILE)) {
            if (!resend_attempted) {
                resend_attempted = true;
                profile_state = ST_SET_PU_PREPROFILE;
            } else {
                resend_attempted = false;
                ZephyrLogWarn("PU not responding to profile command");
                return true;
            }
        }
        break;

    case ST_PREPROFILE_WAIT:
        if (CheckAction(ACTION_END_PREPROFILE)) {
            ZephyrLogFine("Finished docked profile");
            if(pibConfigs.pu_auto_offload.Read())
            {
                Serial.println("Begin Automatic PU Offload");
                SetAction(ACTION_OFFLOAD_PU);
                SetAction(ACTION_OVERRIDE_TSEN);
            }
            return true;
        }
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}