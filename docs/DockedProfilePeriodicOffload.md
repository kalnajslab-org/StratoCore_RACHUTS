# Periodic offload during a docked profile

Status: **implemented** (working tree, not yet committed as of this writing).
Design doc for the `Flight_DockedProfile` periodic-offload change — kept as a
record of the decisions and tradeoffs behind the current implementation.

## Context

Previously a docked profile (TC 153, [Flight_DockedProfile.cpp](../src/Flight_DockedProfile.cpp)) commanded the
RPU into measure for the entire programmed duration, then at the end sent it to standby and handed off
to the offload state machine. Nothing reached the ground until the whole profile was over — a multi-hour
docked profile at a 1 s sample rate accumulates thousands of records with no intermediate downlink, and
the RPU's record FIFO is only 5000 deep (`RPU/src/RPU/RPURecordBuffer.h:19`, `push()` drops silently
when full).

The change: break the measurement into segments. At each segment boundary put the RPU in standby, run
the existing record-offload/TM sequence, then put it back into measure. At the end of the total
programmed length, run the exact same sequence but don't resume measure — which is the original
end-of-profile behavior. The periodic and final paths are literally the same states, differing only by
one boolean.

Decisions confirmed with the user:
- The interval is a **new EEPROM config** `docked_offload_period` (seconds, 0 = disabled) set by a
  **new TC 157**. TC 153's existing 2-param format is unchanged.
- `docked_profile_time` means **total measurement time** — segments sum to it; offload pauses extend
  wall clock beyond it.
- `profile_id` increments **once per docked profile**, not per segment.
- `docked_profile_time == 0` (which the TC parser explicitly allows as "run until commanded to stop",
  but which previously ended the profile after ~2 s) was fixed to mean run indefinitely, offloading each
  period, until TC 156 CANCELMEASURE.

## Design

### Structure: nest `Flight_PUOffload` inside `Flight_DockedProfile`

`Flight_PUOffload(true/false)` is called directly from the docked-profile state machine, the same way
[Flight_PUOffload.cpp:34-41](../src/Flight_PUOffload.cpp#L34-L41) already nests `Flight_CheckPU`. Each
`Flight_*.cpp` owns its file-static state, so nesting is safe, and the docked profile's action flags
(segment/go-measure timing) are disjoint from the offload's (`RESEND_PU_RECORD`, `RESEND_TM`,
`RESEND_PU_CHECK`).

Deliberately **not** bouncing out through `FLM_IDLE` / `ACTION_OFFLOAD_PU` and re-entering: that would
lose the docked profile's static state, need a new "resume" action, and `FLM_IDLE`'s priority if/else
chain ([Flight.cpp:110-146](../src/Flight.cpp#L110-L146)) plus the 3-loop `FLAG_STALE` expiry could let a
stray action silently abandon the profile with the RPU parked in standby.

Consequence accepted: `inst_substate` stays `FLM_DOCKED` for the whole profile, so ground no longer sees
`FLM_PU_OFFLOAD` during a docked profile. The per-segment text TM below replaces that visibility.

### State machine in [Flight_DockedProfile.cpp](../src/Flight_DockedProfile.cpp)

```c
enum ProfileStates_t {
    ST_ENTRY,
    ST_GO_MEASURE,
    ST_CONFIRM_GO_MEASURE,
    ST_MEASURE_WAIT,
    // ---- shared standby+offload sequence (periodic == final) ----
    ST_GO_STANDBY,
    ST_CONFIRM_STANDBY,
    ST_OFFLOAD,
};

static ProfileStates_t profile_state = ST_ENTRY;
static bool     resend_attempted     = false;
static bool     indefinite           = false;  // docked_profile_time == 0
static uint32_t measure_remaining_s  = 0;      // measurement time still owed
static uint16_t segment_len_s        = 0;      // this segment's commanded length (0 = unlimited)
static uint32_t segment_deadline_ms  = 0;
static uint32_t pu_reply_deadline_ms = 0;
static bool     resume_after_offload = false;  // THE bit that distinguishes periodic from final
```

| State | Body | Next |
|---|---|---|
| `ST_ENTRY` | guard `docked_profile_rate != 0`; `indefinite = (docked_profile_time == 0)`; `measure_remaining_s = docked_profile_time`; increment `profile_id` (**moved here** from `ST_CONFIRM_GO_MEASURE` so segments don't bump it); capture `profile_start_lat/lon/alt` from `zephyrRX.zephyr_gps` | fall through |
| `ST_GO_MEASURE` | `segment_len_s = NextSegmentLength(...)` (below); clear `pu_measure`/`pu_standby`; `TX_GoMeasure(segment_len_s, docked_profile_rate, …)`; arm `pu_reply_deadline_ms` | `ST_CONFIRM_GO_MEASURE` |
| `ST_CONFIRM_GO_MEASURE` | on `pu_measure`: `segment_deadline_ms = millis() + segment_len_s*1000 + 2000` (keep the original +2 s head start so the RPU's own timer fires first). On timeout: retry once, else `SendTextTM("RPU not responding to go-measure command, returning to FLM_IDLE", WARN)` + `return true` | `ST_MEASURE_WAIT`, or done |
| `ST_MEASURE_WAIT` | if `segment_len_s == 0` just wait for cancel. Else when `segment_deadline_ms` passes: `measure_remaining_s -= min(remaining, segment_len_s)`; `resume_after_offload = (indefinite \|\| measure_remaining_s > 0)` | `ST_GO_STANDBY` |
| `ST_GO_STANDBY` | `SendTextTM(<segment summary — measured N s, M s remaining, or "final">, FINE)`; clear `pu_standby`; `TX_GoStandby(rpu_bat_temp)`; arm `pu_reply_deadline_ms` | `ST_CONFIRM_STANDBY` |
| `ST_CONFIRM_STANDBY` | on `pu_standby`: `docked_segment++`; `Flight_PUOffload(true)` → `ST_OFFLOAD`. On timeout: retry once, then `SendTextTM("RPU did not confirm standby; aborting docked profile, returning to FLM_IDLE", WARN)` + `return true` — see below | `ST_OFFLOAD`, or done |
| `ST_OFFLOAD` | `if (!Flight_PUOffload(false)) break;` then: offload failed → WARN + `return true` (abort); `!resume_after_offload` → `SendTextTM("Finished docked profile", FINE)` + `return true`; else | `ST_GO_MEASURE` |

Every `return true` above hands control back to `FLM_IDLE`, and each one announces itself:

| Exit | TM | Flag |
|---|---|---|
| `docked_profile_rate == 0` at entry | `Docked profile: invalid rate, returning to FLM_IDLE` | WARN |
| go-measure unacked after 2 attempts | `RPU not responding to go-measure command, returning to FLM_IDLE` | WARN |
| go-standby unacked after 2 attempts | `RPU did not confirm standby; aborting docked profile, returning to FLM_IDLE` | WARN |
| offload failed | `Docked profile aborted: RPU offload failed, returning to FLM_IDLE` | WARN |
| all measurement time used | `Finished docked profile, returning to FLM_IDLE` | FINE |

Both "unacked after 2 attempts" rows mean the same thing — the dock link is down — and both give up
rather than continuing into work that cannot succeed. Each attempt waits `RPU_RECEIVE_TIMEOUT`, so a
dead RPU costs 2 × `RPU_RECEIVE_TIMEOUT` before the profile ends, not the full segment duration.

Segment sizing, as a file-static helper so the rule is stated once:

```c
static uint16_t NextSegmentLength(uint16_t period, uint16_t rate, uint32_t remaining, bool indef)
{
    if (period == 0) return indef ? 0 : (uint16_t)remaining;  // one segment; 0 = unlimited
    if (period <= rate) period = rate + 1;
    if (indef) return period;
    if (remaining <= period) return (uint16_t)remaining;
    if ((remaining - period) <= rate) return (uint16_t)remaining; // fold a sub-sample tail in
    return period;
}
```

Both clamps matter: the RPU silently rewrites a commanded duration when `duration <= rate`
(`RPU/src/RPU/RPU.cpp:215-218`), so an unclamped runt segment would make RACHUTS and the RPU disagree
about how long that segment ran and break the "segments sum to `docked_profile_time`" invariant.

### Timing: local `millis()` deadlines, not `scheduler.AddAction`

This file's use of `RESEND_PU_GOPROFILE` and `ACTION_END_PREPROFILE` was replaced with `millis()`
deadlines (compared with `(int32_t)(millis() - deadline) >= 0` so rollover is handled). The scheduler is
a fixed 32-slot queue with no per-item cancel, `AddAction` failure was unchecked (KnownIssues §15,
now closed), and an offload leaves ~35 `RESEND_TM` entries queued (KnownIssues §3) — so under the new
design the segment timer would have been armed right when the queue is at its fullest, and a silent
`AddAction` failure would hang the profile forever in `ST_CONFIRM_GO_MEASURE`. Local deadlines remove
that coupling entirely. The `ScheduleAction_t` enumerators were left in place; `Flight_Profile.cpp` still
uses both.

### Cancel (TC 156) routes into the shared path

The check stays at the top of the function, before the switch (KnownIssues §14), but is state-aware so a
cancel during an in-flight offload doesn't restart a second one:

```c
if (CheckAction(ACTION_CANCEL_MEASURE)) {
    resume_after_offload = false;      // always: never resume measure after a cancel
    indefinite = false;
    measure_remaining_s = 0;
    if (profile_state == ST_OFFLOAD) {
        log_nominal("Cancel received during offload; finishing current offload");
    } else if (profile_state != ST_GO_STANDBY && profile_state != ST_CONFIRM_STANDBY) {
        profile_state = ST_GO_STANDBY; // same shared path as a normal segment boundary
    }
    return false;                      // stay in FLM_DOCKED until the data is down
}
```

This is idempotent, uses one code path for cancel/periodic/final, and — unlike the original
`SetAction(ACTION_OFFLOAD_PU); return true;` — doesn't risk abandoning the records.
[TCHandler.cpp:273](../src/TCHandler.cpp#L273) still sends `TX_GoStandby` unconditionally when the TC
lands, and the redundant one in `ST_GO_STANDBY` is harmless (the RPU accepts it in any state).

### No text TM while `ST_OFFLOAD` is active

`SendTextTM` ends with `zephyrTX.clearTm()` ([StratoRachuts.cpp:204-216](../src/StratoRachuts.cpp#L204-L216)),
and `Flight_PUOffload`'s `ST_TM_ACK` resend relies on the RPUREPORT TM still living in the XMLWriter
([Flight_PUOffload.cpp:95](../src/Flight_PUOffload.cpp#L95)). Emitting a text TM mid-offload would make a
resend transmit an empty TM. Hence the segment summary is sent in `ST_GO_STANDBY`, before the offload
starts, and the cancel-during-offload branch uses `log_nominal`, not `SendTextTM`.

## Files changed

**Firmware**
- [src/Flight_DockedProfile.cpp](../src/Flight_DockedProfile.cpp) — the rewrite above.
- [src/Flight_PUOffload.cpp](../src/Flight_PUOffload.cpp) — new `pu_offload_success`, set on all three
  `return true` paths (success, PU failure, unknown state), mirroring the existing `check_pu_success`
  convention in [Flight_CheckPU.cpp:46](../src/Flight_CheckPU.cpp#L46). Without it the caller can't tell
  success from failure.
- [src/PURouter.cpp](../src/PURouter.cpp#L56-L62) — in `HandlePUAck` `case RPU_GO_STANDBY`, sets the new
  `pu_standby = true` on ack and clears `pu_measure`; on NAK leaves `pu_standby` false (the existing WARN
  stays). Also clears `pu_standby` in the `RPU_GO_MEASURE` ack so the two edge-triggered flags can't
  cross-contaminate between segments.
- [src/StratoRachuts.h](../src/StratoRachuts.h) — added `bool pu_standby`, `bool pu_offload_success`, and
  `uint8_t docked_segment` members near the existing PU state flags.
- [src/StratoRachuts.cpp](../src/StratoRachuts.cpp#L474-L500) — `SendRPUREPORT` StateDetails2 gained a
  segment field: `"profile:%u segment:%u packet:%u records:%u"`. Needed because `packet_num` restarts
  at 1 in every segment while `profile_id` no longer changes, so `(profile_id, packet_num)` alone would
  not be unique. `docked_segment` is set to 0 by the non-docked (TC 147) offload path.
- [src/TCHandler.cpp](../src/TCHandler.cpp#L240-L252) — new `case SETDOCKEDOFFLOADPERIOD` writing the
  config with a `msg2` echo, plus a `WARN` if `0 < period <= ZEPHYR_RESEND_TIMEOUT` (60 s), since a
  period shorter than the TM resend timeout keeps the scheduler queue permanently loaded. The TC 153 ack
  also now echoes the effective offload period, since it no longer travels with that command.

**EEPROM config** ([src/PIBConfigs.h](../src/PIBConfigs.h) / [src/PIBConfigs.cpp](../src/PIBConfigs.cpp)) —
followed the 3-place rule in the header comment: member, ctor default, `Register()`. `docked_offload_period`
was added **at the end of the list, after `ra_override`** — `SendPIBEEPROM()` dumps raw bytes whose
offsets are defined by `Register()` order, so appending is the only safe position. Default `1800` (30
minutes) — periodic offload is on by default, matching the interval used in the operational cycle example.
`CONFIG_VERSION` bumped `0x5C07` → `0x5C08` so the added field forces a reinitialization to defaults on
first boot (see the note below about this wiping all stored configs).

**New telecommand** (vendored lib, per the established practice of editing `.pio/libdeps` in place) —
`.pio/libdeps/rachuts/StrateoleXML/Telecommand.h`: `SETDOCKEDOFFLOADPERIOD = 157` (156 was the last
RACHUTS id, the RPU block starts at 180) plus `uint16_t dockedOffloadPeriod` in `PIB_Param_t`;
`Telecommand.cpp`: a `ParseTelecommand` case doing `Get_uint16(&(pibParam.dockedOffloadPeriod),1)` —
required, since the `default:` case rejects unknown param-carrying TCs.

> Deployment note: `.pio/` is gitignored and `platformio.ini` pulls StrateoleXML from GitHub, so an
> edit made only in the vendored copy is lost on the next `pio pkg update`. **Resolved** — TC 157 is
> upstream in `kalnajslab-org/StrateoleXML` (`ec162eb`), alongside the two deltas that were previously
> local only: `CANCELMEASURE = 156` (`1f02afb`) and `dockedProfileRate` (`2fd7a86`). The vendored copies
> are byte-identical to upstream and upstream is pushed, so a refetch is safe. The ground TC generator
> still needs 157 registered.

**Docs**
- [TelecommandCribSheet.md](TelecommandCribSheet.md) — row for 157; updated the `duration = 0` note to
  the new meaning; states that TC 153's duration is total *measurement* time and that the period comes
  from EEPROM, not TC 153.
- [FlightModeControlFlow.md](FlightModeControlFlow.md) — rewrote the `Flight_DockedProfile` section and
  mermaid diagram. Also fixed pre-existing staleness found along the way: a reference to
  `pibConfigs.pu_auto_offload`, which no longer existed (removed in commit a9a8849), and a claim that
  `RPU_TM_MAX_RECORDS` is 120 when the RPU actually uses 160 (`RPU/src/RPU/RPU.cpp:67`).
- [KnownIssues.md](KnownIssues.md) — closed §15 (scheduler-hang risk, no longer possible for this file);
  rewrote §14's docked-profile paragraph for the new cancel semantics; corrected §5 (all three of its
  bullets were already fixed upstream in the RPU firmware, unrelated to this change but discovered while
  reading the RPU repo for this work); noted under §3 that periodic offload multiplies its scheduler
  exposure; updated the RPUREPORT row in Appendix A for the segment field.
- [Typical Operational Cycle.md](Typical%20Operational%20Cycle.md) — the `153,…` steps gained a preceding
  `157,<period>`.

## Bench-test finding: an unconfirmed standby aborts the profile

`ST_CONFIRM_STANDBY` originally proceeded into the offload after two unacknowledged `TX_GoStandby`
commands, on the reasoning that the RPU accepts `SEND_RECORDS` from any state so an unconfirmed standby
isn't proof of a problem. Bench testing showed that reasoning to be wrong in practice: if the RPU won't
acknowledge standby, the dock link is down, and the offload cannot succeed either. It merely takes
~40 s of `Flight_CheckPU` and record-request timeouts to arrive at the same failure, emitting a
cascade of WARN TMs along the way:

```
13:17:50  RPU did not confirm standby; offloading anyway
13:18:11  PU not responding to status request           (2 x 10 s)
13:18:32  PU not successful in sending profile record   (2 x 10 s)
13:18:32  Docked profile aborted: RPU offload failed
```

`ST_CONFIRM_STANDBY` now aborts directly on the second timeout with a single WARN and returns to
`FLM_IDLE`, reaching the same outcome immediately and with one message instead of four.

## Bench-test finding: retry bookkeeping (fixed)

Bench testing turned up two related bugs in the retry logic, both stemming from the same root cause:
`resend_attempted` was being unconditionally reset to `false` inside `ST_GO_MEASURE` and `ST_GO_STANDBY`,
including on the very re-entry that a retry causes. Two symptoms:

- The "segment complete" text TM (sent in `ST_GO_STANDBY`) was being duplicated whenever the RPU's
  go-standby ack didn't arrive within `RPU_RECEIVE_TIMEOUT` the first time — observed on the bench as
  two identical `RACHUTSTEXT` messages ~10 s apart before a segment's `RPUREPORT`.
- More importantly, the "retry once, then give up" cap on both the go-measure and go-standby confirm loops
  didn't actually cap at one retry: since the `GO_*` state cleared `resend_attempted` back to `false` right
  after the confirm state had just set it `true`, a second timeout would always be treated as the first,
  looping indefinitely instead of ever reaching the corresponding give-up branch. This pattern predated
  this change (it was already present in the original go-measure retry logic) and was inadvertently
  carried into the new go-standby retry logic when it was added.

Fix: `resend_attempted` is no longer touched inside `ST_GO_MEASURE`/`ST_GO_STANDBY`. Its lifecycle is
owned entirely by the states that transition into a *fresh* attempt (`ST_ENTRY`, the cancel handler's jump
into `ST_GO_STANDBY`, and the confirm states' success paths) and the confirm states' own retry branches,
which is enough to make the retry count actually count. `ST_GO_STANDBY`'s text TM is now also gated on
`!resend_attempted`, so it fires once per segment boundary regardless of how many times the underlying
`TX_GoStandby` has to be resent.

## Notes and known consequences

- **The RPU power-cycles its sensors at every boundary.** `enterStandby()` clears `sensorsEnabled` and
  calls `powerdownSensors()`, so OPC pump / TDLAS / TSEN / RS41 restart cold each segment and the first
  records after a resume may be unsettled. Argues for a large period (900–1800 s), not a small one.
- **`elapsed_s` restarts at 0 each segment** — `enterMeasure()` resets `MeasureStartMillis`. Each block
  header carries a fresh `epoch_time`/`gps_lat`/`gps_lon`, which is the correct stitching key for
  ground processing. Worth telling the ground team along with the segment field.
- **Bumping `CONFIG_VERSION` wipes all EEPROM configs** back to hard-coded defaults, including
  `profile_id` → 1 and `pu_docked` → false. After flashing, the ground must re-send every config TC.
- **`period == 0` with `duration == 0`** measures forever with no drain and silently drops records past
  5000 (~83 min at a 1 s rate). Not mitigated in this change — a candidate for a one-shot WARN text TM
  at `ST_ENTRY` if this configuration turns out to be used in practice.
- Capturing GPS at `ST_ENTRY` also fixed an existing bug: `SendRPUREPORT` reports
  `profile_start_lat/lon/alt`, which were only ever written by `PUStartProfile()`
  ([StratoRachuts.cpp:517-519](../src/StratoRachuts.cpp#L517-L519)) — a path the docked profile never
  took — so previously a docked-profile RPUREPORT carried the last manual profile's position, or zeros.

## Out of scope

Not touched: `Flight_PUOffload`'s scheduler usage (KnownIssues §3 proper), the RPU firmware, and the
`FL_EXIT` gap where a Zephyr-commanded mode switch leaves the RPU in MEASURE with nobody to stop it
(pre-existing; worth a new KnownIssues entry, not a fix here).

## Verification

1. `pio run` — compiles clean for the `rachuts` env. (Done — see build log at implementation time.)
2. **ZephyrSim + RPU (or RPU simulator) on the bench**, the primary test, not yet run:
   - `157,120` then `153,600,10` → expect 5 measure segments of 120 s, each followed by a
     `RACHUTSTEXT` segment summary and one or more `RPUREPORT` TMs with `segment:` incrementing and
     `packet:` restarting; after the 5th, "Finished docked profile" and a return to `FLM_IDLE`.
     Confirm the 5 segment lengths sum to 600 s of *measurement* and that wall clock is longer.
   - `157,0` then `153,300,10` → single segment, one offload, identical to the original behavior.
   - `153,0,10` with `157,120` → runs indefinitely, offloading every 120 s; `156` stops it, and the
     records from the in-progress segment still come down before the mode returns to `FLM_IDLE`.
   - Send `156` *during* an offload → the offload finishes, no second offload starts, no resume.
   - Force an offload failure (RPU unplugged / no response to `RPU_SEND_RECORDS`) mid-profile → WARN
     text TM and the profile aborts rather than resuming measure.
3. Check the RPU console log across a boundary: standby entered, sensors powered down, then measure
   re-entered with the next segment's duration.
4. `152` (GETPIBEEPROM) after `157,900` → `docked_offload_period` reads back as the last field.
5. Watch `scheduler.PrintSchedule()` output (or the debug serial) across two consecutive segments to
   confirm the queue drains during the measure gap and the segment timer no longer depends on it.
