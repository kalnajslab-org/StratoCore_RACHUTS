/*
 *  Flight_DockedProfile.cpp
 *  Author:  Alex St. Clair
 *  Created: June 2020
 */

#include "StratoRatchuts.h"

enum ProfileStates_t {
    ST_ENTRY,
    ST_GET_PU_STATUS,
};

static ProfileStates_t profile_state = ST_ENTRY;
static bool resend_attempted = false;

bool StratoRatchuts::Flight_DockedProfile(bool restart_state)
{
    if (restart_state) profile_state = ST_ENTRY;

    switch (profile_state) {
    case ST_ENTRY:
    case ST_GET_PU_STATUS:
        profile_state = ST_GET_PU_STATUS;
        break;

    default:
        // unknown state, exit
        return true;
    }

    return false; // assume incomplete
}