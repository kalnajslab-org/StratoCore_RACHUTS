# Typical daily operating schedule for RACHuTS in fully operational flight

> TC numbers/names per `docs/TelecommandCribSheet.md`. `MANUALPROFILE`,
> `DOCKEDPROFILE`, and `RPUCONFIG` run in flight mode. Autonomous mode and the
> `SETAUTO`/`SETMANUAL` selection were removed — the original version of this doc
> called for `131 SETMANUAL` to be sent, but with those removals it is not
> available nor needed.

| # | Segment | Duration | Action | Telecommand(s) |
|---|---------------|----------|--------|----------------|
| 1 | Initiate | — | Go Flight — Zephyr IM message (Mode = FL) | — |
| 2 | Configure | — | Set preprofile time to 60s | `148,60` |
| 3 | 1st Docked Measure | 8 hours | Set RATCHUTSREPORT cadence to 1 hour, then docked measurement, 8 hours, 30s measurement cadence, only TSEN and RSS421 on | `181,3600` → `180,28800,30,0,0,1,1` → `153,28800` |
| 4 | 1st Profile Measure | ~11.5 min (est.) | Set RATCHUTSREPORT cadence to 10 minutes, then profile, all instruments on, length 1000, dock 100, overshoot 50, dwell 120s | `181,600` → `146,1000,100,50,120` |
| 5 | 2nd Profile Measure | ~40.4 min (est.) | Set RATCHUTSREPORT cadence to 10 minutes, then profile, all instrumetns on, length 4000, dock 250, overshoot 100, dwell 300s | `181,600` → `146,4000,250,100,300` |
| 6 | 2nd Docked Measure | 2 hours | Set RATCHUTSREPORT cadence to 1 hour, then docked measurement, 2 hours, 10s measurement cadence, all sensors on | `181,3600` → `180,7200,10,1,1,1,1` → `153,7200` |
| 7 | 3rd Profile Measure | ~1h 13min (est.) | Set RATCHUTSREPORT cadence to 10 minutes, then profile, all instruments on, length 7000, dock 500, overshoot 200, dwell 600s | `181,600` → `146,7000,500,200,600` |
| 8 | Reset RATCHUTSREPORT cadence | — | Set RATCHUTSREPORT cadence back to 1 hour | `181,3600` |
| 9 | Standby | — | Go standby — Zephyr IM message (Mode = SB) | — |
| 10 | Sleep | — | Sleep for 2 hours | — |

Estimated profile durations = reel-out + dwell + reel-in + dock time, computed
as `revolutions / velocity (rpm) × 60` per leg (the same basis
`StartMCBMotion()` uses for its motion-timeout budget), using the default
velocities (deploy/retract 250 rpm, dock 80 rpm) and each profile's derived
`retract_length = size − dock_amount` / `dock_length = dock_amount +
overshoot`. These are estimates, not guarantees — they exclude PU handshake
time, any redock retries, and assume the default velocities haven't been
changed via `DEPLOYv`/`RETRACTv`/`DOCKv`.
