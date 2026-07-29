# Typical daily operating schedule for RACHuTS in fully operational flight

> TC numbers/names per `Documentation/TelecommandCribSheet.md`. `MANUALPROFILE`,
> `DOCKEDPROFILE`, and `RPUCONFIG` run in flight mode. Autonomous mode and the
> `SETAUTO`/`SETMANUAL` selection were removed — the original version of this doc
> called for `131 SETMANUAL` to be sent, but with those removals it is not
> available nor needed.

| # | Action | Telecommand(s) |
|---|--------|----------------|
| 1 | Go Flight — Zephyr IM message (Mode = FL) | — |
| 2 | Set preprofile time to 60s | `148,60` |
| 3 | Docked measurement, 8 hours, 30s measurement cadence, only TSEN and RSS421 on | `180,28800,30,0,0,1,1` → `153,28800` |
| 4 | Profile, all instruments on, length 1000, dock 100, overshoot 50, dwell 120s | `146,1000,100,50,120` |
| 5 | Profile, all instrumetns on, length 4000, dock 250, overshoot 100, dwell 300s | `146,4000,250,100,300` |
| 6 | Docked measurement, 2 hours, 10s measurement cadence, all sensors on | `180,7200,10,1,1,1,1` → `153,7200` |
| 7 | Profile, all instruments on, length 7000, dock 500, overshoot 200, dwell 600s | `146,7000,500,200,600` |
| 8 | Go standby — Zephyr IM message (Mode = SB) | — |
| 9 | Sleep for 2 hours | — |
