# Motion Fixes Hardware Test Plan

Bench tests for the motion and profile fixes on the `Motion_timeout_fix` branch
(and the Zero Reel fix in `00274d6`). Written 2026-10-04.

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

## Setup

- Bench setup with the Zephyr simulator, MCB and RPU on the dock, watching TMs
  and the PIB debug serial.
- Use short settings so each run takes minutes: for example a 50-rev profile, a
  30 s dwell, and a 20–30 s pre-profile time (`SETPREPROFILETIME`).
- For runs where the PU needs to leave the dock, use a deploy length that stays
  within the bench's travel.
- Several tests depend on timing, so note the timestamps of the MCBREPORT and
  RACHUTSTEXT TMs.

## A. Zero Reel

- [ ] **1. The original bug.** Run a TC 142 redock that ends normally, then
  send Zero Reel. **Expect** "MCB acked zero reel" with no "motion ongoing" WARN.
- [ ] **2.** Send Zero Reel during a manual reel-out. **Expect** WARN "Can't
  zero reel, motion ongoing".
- [ ] **3.** Send Zero Reel during a profile dwell. **Expect** WARN "Can't zero
  reel, profile in progress", and the reel position is unchanged.

## B. Motion timeouts and dock wait

- [ ] **4. Normal profile.** **Expect** the dock to start about 60 s after
  "Finished profile reel in". It used to be about 30 s.
- [ ] **5. Short dwell, which used to cause a false fault.** Set the dwell to
  about 5 s, leaving `motion_timeout` at 30. **Expect** the profile to complete
  with no "took longer than expected".
- [ ] **6. Large timeout, which used to abort the dock.** Set `motion_timeout`
  to 90 and run a profile. **Expect** it to complete normally.
- [ ] **7. The timeout still works.** Set `motion_timeout` to 0 and send a
  manual deploy. **Expect** CRIT "MCB Motion took longer than expected", an MCB
  cancel and the error state. The acceleration ramps make the motion take longer
  than its planned time, so it should trip. Repeat with TC 142; the redock had
  no timeout before, so this one is new. Afterwards, send `EXITERROR` and
  restore the timeout.

## C. Cancel Motion

### Profile

- [ ] **8.** Cancel during the pre-profile wait. **Expect** WARN "Profile
  cancelled before reel out", the RPU in standby (check with TC 143), and a
  return to idle.
- [ ] **9.** Right after test 8, start another profile. **Expect** its
  pre-profile wait to run its full length, showing no leftover timer cut it
  short.
- [ ] **10.** Cancel during the reel-out. **Expect** the error state and the MCB
  in low power.
- [ ] **11.** Cancel during the dwell, and separately during the dock wait.
  **Expect** the error state. Before the fix, both were ignored.
- [ ] **12.** Cancel during the offload at the end of a profile. **Expect**
  WARN "no motion to cancel, profile motion already complete", and the offload
  finishes.

### Manual motion

- [ ] **13.** Cancel while waiting for the RA ack. Hold the ack in the
  simulator, with the RA override off. **Expect** WARN "Manual motion cancelled
  before start", and the motion never starts after the ack.
- [ ] **14.** Cancel during a manual reel-out, then immediately send another
  manual motion. **Expect** "Commanded motion stop", then the second motion runs
  normally.

### Redock (TC 142)

- [ ] **15.** Cancel between the reel-out and the reel-in. **Expect** the error
  state. Before the fix, this was ignored.
- [ ] **16.** Cancel during the final PU check. **Expect** WARN "no motion to
  cancel, redock motion already complete", and the check finishes.

### No motion

- [ ] **17.** Cancel in flight idle, and again in standby mode. **Expect** WARN
  "Cancel motion: no motion to cancel".
- [ ] **18. Recovery path.** After a cancel puts the PIB in the error state,
  send `EXITERROR` and then a manual retract. **Expect** the retract to run.

## D. Commands refused while busy

- [ ] **19.** During a profile, send DEPLOYx, RETRACTx, DOCKx, 142, 143, 146,
  147 and 153. **Expect** a WARN for each, "<cmd> ignored: profile in progress".
  - **Check:** the profile's reel-in length is unchanged; the reel position
    should return to its expected value.
  - **Check:** a 146 sent with a different dwell doesn't change the running
    dwell.
- [ ] **20.** In the error state, send the same TCs. **Expect** "... ignored: in
  flight error state (send EXITERROR)". After `EXITERROR` they should work.
- [ ] **21.** In standby mode, send DEPLOYx. **Expect** "Deploy ignored: not in
  flight mode".
- [ ] **22.** Send a DEPLOYx during a manual motion and during a docked profile.
  **Expect** "manual motion in progress" and "docked profile in progress".

## E. Redock timing and limit

- [ ] **23. Short reel-out.** TC 142 with about 5 revs. **Expect** the reel-in
  at about +30 s and the PU check at about +60 s, the same as before.
- [ ] **24. The original TC 142 bug.** TC 142 with a reel-out that takes more
  than 30 s; lower DEPLOYv if needed. **Expect** the reel-in to start right
  after the reel-out finishes, and the PU check right after the reel-in, now
  that it's past +60 s.
- [ ] **25. Redock limit.** Set `num_redock` to 1, then make the dock check fail
  during a profile: power off the RPU or disconnect the dock serial line after
  the dock motion. **Expect** one redock, then CRIT "No dock! Exceeded allowable
  number of redock attempts".
- [ ] **26. Limit lowered mid-sequence.** Set `num_redock` to 3, and once the
  first redock starts send AUTOREDOCKPARAMS with the count set to 0. **Expect**
  CRIT at the next failed dock check.

## F. Regression

- [ ] **27.** One full normal profile and one docked profile with the default
  settings, end to end, including the offload.
- [ ] **28. Safety mode.** Check that the full retract and dock still complete.
  A timeout that Safety mode never checked was removed there.
