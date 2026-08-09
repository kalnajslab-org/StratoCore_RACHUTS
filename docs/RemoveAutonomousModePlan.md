# Plan: Remove Autonomous Mode from RACHUTS

Status: **planning** (no code changed yet). Goal: remove autonomous flight mode
from the RACHUTS PIB, leaving manual flight as the only flight path.

## Settled decisions
- **Config strategy: remove the 5 autonomous configs and bump `CONFIG_VERSION`.**
  Nothing has been deployed, so there is no operator EEPROM state to preserve —
  pitfall #1 does not apply. (Configs reload defaults on first boot; fine.)
- **Remove the `SETAUTO`/`SETMANUAL` TC handling.**
- **Docked profile is unaffected** — it shares no config or TC with autonomous
  mode and lives on the manual (`FLM_*`) path; the later docked-profile refactor
  is independent.

## What "autonomous mode" comprises

Three coupled pieces:
- **The `FLA_*` flight substates** + `AutonomousFlight()` state machine.
- **`SETAUTO`/`SETMANUAL`** and the **SZA/time-trigger + profile-scheduling**
  config those drive.
- **The `autonomous_mode` flag** that branches the flight loop and gates motion TCs.

Default is already `autonomous_mode = false` (manual), so removal deletes the
*alternate* path — the nominal path is unchanged.

---

## Sections that change

### 1. `Flight.cpp` — the core
- **`FLStates_t` enum**: delete `FLA_IDLE, FLA_WAIT_PROFILE, FLA_PROFILE,
  FLA_PU_OFFLOAD, FLA_NOTE_PROFILE_END`. **Pin the survivors** (pitfall #2).
- **`FL_GPS_WAIT`**: `inst_substate = (autonomous_mode) ? FLA_IDLE : FLM_IDLE;`
  becomes `FLM_IDLE`.
- **`FL_ENTRY`**: drop `profiles_remaining = 0;`.
- **Dispatch default**: drop the `if (autonomous_mode) AutonomousFlight()` branch
  and call `ManualFlight()` directly (or inline it).
- **`AutonomousFlight()`**: delete entirely. `ManualFlight()` stays (optionally
  fold into `FlightMode`).

### 2. `TCHandler.cpp`
- Delete **`SETAUTO`** / **`SETMANUAL`** cases.
- Delete autonomous-config cases: **`SETSZAMIN`, `SETPROFILEPERIOD`,
  `SETNUMPROFILES`, `USESZATRIGGER`, `USETIMETRIGGER`, `SETTIMETRIGGER`**.
- **`RequireManualFlight`**: drop the `autonomous_mode` check — it collapses to
  "am I in FL mode?" Rename to `RequireFlightMode` for honesty.
- **`DEPLOYx`/`RETRACTx`/`DOCKx`**: remove the `if (autonomous_mode)` "switch to
  manual" guard — motion is unconditionally allowed in flight.

### 3. `StratoRachuts.h`
- Remove members: `autonomous_mode`, `profiles_remaining`, `profiles_scheduled`.
- Remove decls: `AutonomousFlight()`, `ScheduleProfiles()`.
- `ScheduleAction_t`: remove `ACTION_BEGIN_PROFILE` (autonomous-only; consumed
  only in `FLA_WAIT_PROFILE`).

### 4. `StratoRachuts.cpp`
- Delete **`ScheduleProfiles()`**.
- **`StartMCBMotion()`**: `if (autonomous_mode) log_nominal(...) else
  SendTextTM(...)` becomes just `SendTextTM(...)`.

### 5. `PIBConfigs.h/.cpp` — autonomous-only configs
Remove (and their `Register()` + ctor initializers): `sza_minimum`,
`time_trigger`, `sza_trigger`, `num_profiles`, `profile_period`.
**EEPROM layout** — see pitfall #1.

### 6. `Telecommand.h` / `Telecommand.cpp` (vendored `StrateoleXML`)
The `SETAUTO…SETTIMETRIGGER` enum IDs + `pibParam` parse cases. **Shared enum** —
see pitfall #3. Recommendation: **leave the IDs reserved**, don't renumber.

### 7. Docs
`TelecommandCribSheet.md` (remove auto TCs + autonomous sequences),
`KnownIssues.md` §11 (its wording leans on `autonomous_mode` and substate 14),
and the `MEMORY.md` notes.

---

## Explicitly KEEP (misleadingly "auto"-named, but NOT autonomous flight)
- **`AUTOREDOCKPARAMS` (150)** + `num_redock`/`redock_out`/`redock_in` — used by
  *manual* redock (`Flight_Profile.cpp:148`).
- **`ra_override`** — manual-motion RA bypass.
- **`SETPROFILESIZE/DOCKAMOUNT/DWELLTIME/DOCKOVERSHOOT`**, `preprofile_time`,
  `puwarmup_time`, `motion_timeout` — shared with manual profiles.
- **`pu_auto_offload`** — verify, but appears independent of autonomous *flight*.

---

## Pitfalls (ranked)

1. **EEPROM layout break (highest risk).** `PIBConfigs` uses
   `TeensyEEPROM(CONFIG_VERSION = 0x5C05)` and a fixed `Register()` order.
   Deleting fields shifts every field after them. **You must bump
   `CONFIG_VERSION`** (`PIBConfigs.h:28`) so the mismatch forces a reload of
   defaults — otherwise a flashed board reads garbage for all later configs.
   Consequence: **all configs reset to defaults on first boot; operators must
   re-send config TCs.** *Lower-risk alternative:* leave the 5 config fields in
   place (unused) and only delete the code that references them — no version
   bump, no operator reconfig.

2. **Substate renumbering (operator-facing).** `FLA_*` sit *between* `FLM_*` and
   `FL_ERROR_LOOP`/`FL_SHUTDOWN_LOOP`. Deleting 5 entries shifts **`FL_ERROR_LOOP`
   from 14 to 9**. `SENDSTATE` reports the raw number and runbooks/§11 say
   "substate 14 = error loop." **Mitigation:** explicitly pin values —
   `FL_ERROR_LOOP = 14, FL_SHUTDOWN_LOOP = 15` — so the numbers survive deletion.
   *(Update, 2026-07-31: unpinned. Nothing in the firmware itself depended on the
   number, the system is pre-deployment, and the only justification was operator
   familiarity — judged not worth the standing exception. `FL_ERROR_LOOP` is now
   9, `FL_SHUTDOWN_LOOP` is 10. See `FlightModeControlFlow.md`.)*

3. **Shared vendored `Telecommand.h`.** That enum is common to RATS/DIB/ground.
   **Do not remove or renumber** the IDs — leave `SETAUTO=130 … SETTIMETRIGGER=140`
   reserved. Just stop handling them in RACHUTS.

4. **Deprecated TCs hitting `default` -> `CRIT "Unknown TC"`.** If the ground
   still sends `SETAUTO`/`SETMANUAL`, the RATS-style default emits a `CRIT` ack.
   Consider keeping thin cases that reply `WARN "autonomous mode removed"` instead
   of a scary CRIT, until the ground is updated.

5. **Loss of `SETMANUAL` as a flight-restart lever.** Operators sometimes send
   `SETMANUAL` to bounce the flight state machine (it sets `MODE_ENTRY`). After
   removal that trick is gone — `EXITERROR`/`RESET_INST` remain.

6. **Ground software sync (out of repo).** `TCMessage.py` / operator tooling must
   stop offering the removed TCs. Firmware change alone won't prevent them being
   sent.

---

## Suggested order & verification
1. Gut `Flight.cpp` (enum + dispatch + delete `AutonomousFlight`), pinning
   `FL_ERROR_LOOP`. Build.
2. `TCHandler` (remove cases, simplify guard, drop motion guards). Build.
3. Remove members + `ScheduleProfiles` + `ACTION_BEGIN_PROFILE`. Build.
4. **Decide the config strategy** (leave-in-place vs remove + `CONFIG_VERSION`
   bump) before touching `PIBConfigs`.
5. Docs + ground.
6. Bench test: `SB -> FL` enters `FLM_IDLE`; manual TCs (142/143/146/147/153)
   still work; a deprecated TC yields a sane ack; `SENDSTATE` substate numbers
   unchanged.

## Open decision
**Config strategy (step 4)** is the main fork:
- **Leave-in-place** — zero EEPROM risk, a little dead data.
- **Remove + bump `CONFIG_VERSION`** — clean, but resets all operator config on flash.
