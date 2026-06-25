# RACHUTS Telecommand Crib Sheet

Quick reference for the telecommands RACHUTS (PIB) accepts. Numbers are the TC
id. "Params" lists the ordered parameters the command expects; commands with no
Params take none. Source of truth: `StrateoleXML/Telecommand.h` (enum) and
`src/TCHandler.cpp` (handlers). Last updated 2026-06-15.

> Only the commands below are handled by RACHUTS. TC ranges for other
> instruments — **50–57** (FTR/DIB), **60–76** (RATS/ECU), **100–119** (PHA) —
> are *not* accepted by RACHUTS and will be NAK'd.

---

## MCB / reel motion (1–12)

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 1 | DEPLOYx | Reel out | deploy length (rev) |
| 2 | DEPLOYv | Set deploy velocity | velocity (rev/s) |
| 3 | DEPLOYa | Set deploy acceleration | accel (rev/s²) |
| 4 | RETRACTx | Reel in | retract length (rev) |
| 5 | RETRACTv | Set retract velocity | velocity (rev/s) |
| 6 | RETRACTa | Set retract acceleration | accel (rev/s²) |
| 7 | DOCKx | Set dock length | dock length (rev) |
| 8 | DOCKv | Set dock velocity | velocity (rev/s) |
| 9 | DOCKa | Set dock acceleration | accel (rev/s²) |
| 10 | FULLRETRACT | Full retract | — |
| 11 | CANCELMOTION | Cancel any ongoing motion | — |
| 12 | ZEROREEL | Zero the reel position | — |

## Motion limits / MCB (13–18)

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 13 | TEMPLIMITS | Set MCB temperature limits | 6 MCB-specific values |
| 14 | TORQUELIMITS | Set MCB torque limits | 2 MCB-specific values |
| 15 | CURRLIMITS | Set MCB current limits | 2 MCB-specific values |
| 16 | IGNORELIMITS | Ignore limits | — |
| 17 | USELIMITS | Enable limits | — |
| 18 | GETMCBEEPROM | Request MCB EEPROM (returned as a TM) | — |

## Flight mode (130–131)

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 130 | SETAUTO | Switch to autonomous flight mode (restarts flight mode) | — |
| 131 | SETMANUAL | Switch to manual flight mode (restarts flight mode) | — |

## Autonomous profile configuration (132–141)

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 132 | SETSZAMIN | Min SZA for autonomous profile trigger | SZA (float) |
| 133 | SETPROFILESIZE | Profile deploy length | size (float, rev) |
| 134 | SETDOCKAMOUNT | Dock retract length | amount (float, rev) |
| 135 | SETDWELLTIME | Dwell time at profile bottom | time (uint16, s) |
| 136 | SETPROFILEPERIOD | Period between autonomous profiles | period (uint16, s) |
| 137 | SETNUMPROFILES | Profiles per autonomous session | count (uint) |
| 138 | USESZATRIGGER | Trigger autonomous profiles on SZA threshold | — |
| 139 | USETIMETRIGGER | Trigger autonomous profiles on time | — |
| 140 | SETTIMETRIGGER | Unix timestamp for autonomous trigger | timestamp (uint32) |
| 141 | SETDOCKOVERSHOOT | Dock overshoot distance | overshoot (float, rev) |

## Manual profile / dock operations

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 142 | RETRYDOCK | Manual redock (**manual only**) | deploy len (rev), retract len (rev) |
| 146 | MANUALPROFILE | Execute a profile (**manual only**) | profile size (rev), dock amount (rev), dock overshoot (rev), dwell (s) |
| 147 | OFFLOADPUPROFILE | Offload stored RPU profile data (**manual only**) | — |
| 148 | SETPREPROFILETIME | Pre-profile wait after RPU enters measure | time (uint16, s) |
| 149 | SETPUWARMUPTIME | PU warmup time | time (uint16, s) |
| 150 | AUTOREDOCKPARAMS | Auto-redock parameters | redock out (rev), redock in (rev), max retries |
| 151 | SETMOTIONTIMEOUT | Motion timeout | timeout (uint16, s) |
| 153 | DOCKEDPROFILE | Execute a docked profile (**manual only**) | duration (s) |
| 154 | STARTREALTIMEMCB | Enable real-time MCB data streaming | — |
| 155 | EXITREALTIMEMCB | Disable real-time MCB data streaming | — |

## RPU (Profiler) dock control

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 143 | GETPUSTATUS | Request RPU status over dock (**manual only**) → RPUSTATUS TM | — |
| 144 | PUPOWERON | Enable RPU dock power | — |
| 145 | PUPOWEROFF | Disable RPU dock power | — |
| 180 | RPUCONFIG | Configure RPU measurement (stored) | duration (s), rate (s), ROPC, TDLAS, TSEN, RS41 |
| 181 | RPUSTATUSPERIOD | RPU status report period | period (uint16, s) |
| 182 | RPUBATTEMP | RPU battery temperature threshold | temp (float, °C) |
| 183 | RPURESET | Reboot the RPU via dock serial | — |
| 184 | RPUGOSTANDBY | Command RPU to STANDBY | — |
| 185 | RPUGOMEASURE | Command RPU to MEASURE | duration (s), rate (s) |

Notes:
- **RPUCONFIG / RPUGOMEASURE validation:** `rate` must be > 0; a nonzero
  `duration` must be greater than `rate` (else the TC is NAK'd). `duration = 0`
  means "run until commanded to STANDBY / record buffer full."
- **TC 185 (RPUGOMEASURE)** now carries `duration`/`rate` directly; battery
  setpoint and sensor-enable flags come from the stored RPUCONFIG.

## Diagnostics / EEPROM

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 18 | GETMCBEEPROM | MCB EEPROM as a TM | — |
| 152 | GETPIBEEPROM | PIB/RACHUTS EEPROM as a TM (refused during motion) | — |

## General (StratoCore built-ins)

| TC | Name | Description | Params |
|----|------|-------------|--------|
| 200 | RESET_INST | Reboot the PIB (Teensy) | — |
| 201 | EXITERROR | Exit the error state | — |
| 202 | GETTMBUFFER | Re-send the last TM buffer | — |
| 203 | SENDSTATE | Report current mode/substate as a log message | — |

---

## Common sequences

- **Get RPU status (manual):** `143` → RPUSTATUS TM.
- **Bench RPU measure + offload:** `180` (config) → `185` (go-measure, with
  duration/rate) → wait → `184` (standby) → `147` (offload) → RPUREPORT TMs.
- **Manual profile:** `131` (manual) → `146` (with size/dock/overshoot/dwell).
- **Dump configs:** `18` (MCB EEPROM), `152` (PIB EEPROM).
