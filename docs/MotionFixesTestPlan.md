# Motion Fixes Hardware Test Plan

Bench tests for the motion and profile fixes on the `Motion_timeout_fix` branch
(and the Zero Reel fix in `00274d6`). Written 2026-10-04, TC numbers added 2026-10-05.

Changes covered:

- Zero Reel TC checks `mcb_motion_ongoing`; `mcb_dock_ongoing` is cleared when a
  motion finishes.
- Motion timeouts use `millis()` deadlines instead of scheduled actions; the
  dock wait is a fixed 60 s (`DOCK_WAIT_TIME`); redock motions now have a
  timeout.
- Cancel Motion (TC 11) is acted on in every state of `Flight_Profile`,
  `Flight_ManualMotion` and `Flight_ReDock`, and a WARN is sent when there is
  nothing to cancel.
- TCs that start a sequence or set its lengths are rejected with a WARN unless
  flight mode is idle (`RequireFlightIdle`).
- Redock steps run in sequence (reel-in no earlier than +30 s, PU check no
  earlier than +60 s) instead of on fixed scheduled times.
- Redock limit uses `>` and a wider counter.

Highest priority: tests 1, 4, 11, 19 and 24. They cover the original bugs and
the changes most likely to affect a flight.

## TCs used

| TC | Name | Params |
|----|------|--------|
| 1 | DEPLOYx | deploy length (rev) |
| 2 | DEPLOYv | velocity (rev/s) |
| 4 | RETRACTx | retract length (rev) |
| 7 | DOCKx | dock length (rev) |
| 11 | CANCELMOTION | — |
| 12 | ZEROREEL | — |
| 142 | RETRYDOCK | deploy len (rev), retract len (rev) |
| 143 | GETPUSTATUS | — |
| 146 | PROFILE | profile size (rev), dock amount (rev), dock overshoot (rev), dwell (s), RPU sample rate (s) |
| 147 | OFFLOADPUPROFILE | — |
| 148 | SETPREPROFILETIME | time (s) |
| 150 | AUTOREDOCKPARAMS | redock out (rev), redock in (rev), max retries |
| 151 | SETMOTIONTIMEOUT | timeout (s) |
| 153 | DOCKEDPROFILE | duration (s), rate (s) |
| 159 | RAACKOVERRIDEOFF | — |
| 201 | EXITERROR | — |
| 203 | SENDSTATE | — |

Mode changes (standby, flight, safety) are commanded from the Zephyr
simulator, not by TC. TC 203 (SENDSTATE) is handy for confirming the mode and
substate after a test (flight idle and the flight error state).

## Setup

- Bench setup with the Zephyr simulator, MCB and RPU on the dock, watching TMs
  and the PIB debug serial.
- Use short settings so each run takes minutes: for example a 50-rev profile
  with a 30 s dwell (both set in the TC 146 parameters), and a 20–30 s
  pre-profile time (TC 148).
- Note the default `motion_timeout` (30 s, TC 151) and `num_redock` (3, TC 150)
  so you can restore them after the tests that change them.
- For runs where the PU needs to leave the dock, use a deploy length that stays
  within the bench's travel.
- Several tests depend on timing, so note the timestamps of the MCBREPORT and
  RACHUTSTEXT TMs.

## A. Zero Reel

- [ ] **1. The original bug.** Run a redock (TC 142) that ends normally, then
  send Zero Reel (TC 12). **Expect** "MCB acked zero reel" with no "motion ongoing" WARN.
- [ ] **2.** Send Zero Reel (TC 12) during a manual reel-out (TC 1). **Expect** WARN "Can't
  zero reel, motion ongoing".
- [ ] **3.** Send Zero Reel (TC 12) during a profile (TC 146) dwell. **Expect** WARN "Can't zero
  reel, profile in progress", and the reel position is unchanged.

## B. Motion timeouts and dock wait

- [ ] **4. Normal profile.** Run a profile (TC 146). **Expect** the dock to start about 60 s after
  "Finished profile reel in". It used to be about 30 s.
- [ ] **5. Short dwell, which used to cause a false fault.** Run a profile
  (TC 146) with a dwell of about 5 s, leaving `motion_timeout` at 30. **Expect** the profile to complete
  with no "took longer than expected".
- [ ] **6. Large timeout, which used to abort the dock.** Set `motion_timeout`
  to 90 (TC 151) and run a profile (TC 146). **Expect** it to complete normally.
- [ ] **7. The timeout still works.** Set `motion_timeout` to 0 (TC 151) and
  send a manual deploy (TC 1). **Expect** CRIT "MCB Motion took longer than expected", an MCB
  cancel and the error state. The acceleration ramps make the motion take longer
  than its planned time, so it should trip. Repeat with TC 142; the redock had
  no timeout before, so this one is new. After each, send EXITERROR (TC 201).
  Afterwards, restore the timeout (TC 151).

## C. Cancel Motion

### Profile

- [ ] **8.** Start a profile (TC 146) and cancel (TC 11) during the
  pre-profile wait. **Expect** WARN "Profile
  cancelled before reel out", the RPU in standby (check with TC 143), and a
  return to idle.
- [ ] **9.** Right after test 8, start another profile (TC 146). **Expect** its
  pre-profile wait to run its full length, showing no leftover timer cut it
  short.
- [ ] **10.** Cancel (TC 11) during the profile reel-out. **Expect** the error state and the MCB
  in low power.
- [ ] **11.** Cancel (TC 11) during the dwell, and separately during the dock
  wait. Send EXITERROR (TC 201) after each.
  **Expect** the error state. Before the fix, both were ignored.
- [ ] **12.** Cancel (TC 11) during the offload at the end of a profile. **Expect**
  WARN "no motion to cancel, profile motion already complete", and the offload
  finishes.

### Manual motion

- [ ] **13.** Send a manual deploy (TC 1) and cancel (TC 11) while
  it waits for the RA ack. Hold the ack in the simulator, with the RA override
  off (TC 159). **Expect** WARN "Manual motion cancelled
  before start", and the motion never starts after the ack.
- [ ] **14.** Cancel (TC 11) during a manual reel-out (TC 1), then
  immediately send another manual motion (TC 4). **Expect** "Commanded motion stop", then the second motion runs
  normally.

### Redock (TC 142)

- [ ] **15.** Start a redock (TC 142) and cancel (TC 11) between the reel-out
  and the reel-in. **Expect** the error
  state. Before the fix, this was ignored.
- [ ] **16.** Cancel (TC 11) during the redock's final PU check. **Expect** WARN "no motion to
  cancel, redock motion already complete", and the check finishes.

### No motion

- [ ] **17.** Cancel (TC 11) in flight idle, and again in standby mode. **Expect** WARN
  "Cancel motion: no motion to cancel".
- [ ] **18. Recovery path.** After a cancel puts the PIB in the error state,
  send EXITERROR (TC 201) and then a manual retract (TC 4). **Expect** the retract to run.

## D. Commands refused while busy

- [ ] **19.** During a profile (TC 146), send TCs 1, 4, 7, 142, 143, 146,
  147 and 153. **Expect** a WARN for each, "<cmd> ignored: profile in progress".
  - **Check:** the profile's reel-in length is unchanged; the reel position
    should return to its expected value.
  - **Check:** a TC 146 sent with a different dwell doesn't change the running
    dwell.
- [ ] **20.** In the error state, send the same TCs. **Expect** "... ignored: in
  flight error state (send EXITERROR)". After EXITERROR (TC 201) they should work.
- [ ] **21.** In standby mode, send a deploy (TC 1). **Expect** "Deploy ignored: not in
  flight mode".
- [ ] **22.** Send a deploy (TC 1) during a manual motion (TC 4) and during a
  docked profile (TC 153).
  **Expect** "manual motion in progress" and "docked profile in progress".

## E. Redock timing and limit

- [ ] **23. Short reel-out.** Redock (TC 142) with about 5 revs out. **Expect** the reel-in
  at about +30 s and the PU check at about +60 s, the same as before.
- [ ] **24. The original TC 142 bug.** Redock (TC 142) with a reel-out that takes
  more than 30 s; lower the deploy velocity (TC 2) if needed, and restore it
  afterwards. **Expect** the reel-in to start right
  after the reel-out finishes, and the PU check right after the reel-in, now
  that it's past +60 s.
- [ ] **25. Redock limit.** Set `num_redock` to 1 (TC 150), then make the dock
  check fail during a profile (TC 146): power off the RPU or disconnect the dock serial line after
  the dock motion. **Expect** one redock, then CRIT "No dock! Exceeded allowable
  number of redock attempts".
- [ ] **26. Limit lowered mid-sequence.** Set `num_redock` to 3 (TC 150), and once the
  first redock starts send TC 150 again with the count set to 0. **Expect**
  CRIT at the next failed dock check.

## F. Regression

- [ ] **27.** One full normal profile (TC 146) and one docked profile (TC 153)
  with the default settings, end to end, including the offload.
- [ ] **28. Safety mode.** Command safety mode from the Zephyr simulator and check that the full
  retract and dock still complete.
  A timeout that Safety mode never checked was removed there.
