# Periodic offload during a docked profile

Status: **implemented**. Design doc for the `Flight_DockedProfile` periodic-offload rework — kept as a
record of the decisions and tradeoffs behind the implementation.

## Context

Previously a docked profile (TC 153, [Flight_DockedProfile.cpp](../src/Flight_DockedProfile.cpp)) commanded the
RPU into measure for the entire programmed duration, then at the end sent it to standby and handed off
to the offload state machine. Nothing reached the ground until the whole profile was over — a multi-hour
docked profile at a fast sample rate accumulates thousands of records with no intermediate downlink, and
the RPU's record FIFO is only 5000 deep (`RPU/src/RPU/RPURecordBuffer.h`, `push()` drops silently when
full).

The change: break the measurement into periods. At each period boundary, put the RPU in standby, run
the existing record-offload/TM sequence, then put it back into measure. At the end of the total
programmed length, run the exact same sequence but don't resume measure — the original end-of-profile
behavior. The periodic and final paths are the same states, differing only by one boolean.

**Terminology**: a 160-record TM chunk is already conventionally called a "segment"/block (matching
`SendRPUREPORT`'s `packet_num` and the offload's own `"Profile block %u"` log line). To avoid colliding
with that, one docked-profile measure-then-offload cycle is called a **"period"** throughout — matching
the `docked_offload_period` EEPROM config name and the `NextPeriodLength()` helper's own `period`
parameter.

Decisions:
- The interval is a **EEPROM config** `docked_offload_period` (seconds, default 1800s/30min, 0 =
  disabled) set by **TC 157** (`SETDOCKEDOFFLOADPERIOD`). TC 153's existing 2-param format is unchanged.
- `docked_profile_time` means **total measurement time** — periods sum to it; offload pauses extend
  wall clock beyond it.
- `profile_id` increments **once per docked profile**, not per period.
- **A docked profile must always have a finite duration** — `docked_profile_time == 0` is rejected
  outright, both by the TC parser (`DOCKEDPROFILE` validation) and defensively by `ST_ENTRY`.

  *Design-error correction (caught 2026-08-19):* an earlier iteration let `docked_profile_time == 0`
  mean "run indefinitely until TC 156." That semantic traces to `RPUGOMEASURE` (TC 185, `Telecommand.h`
  marks it *"Development testing only — not used in flight operations"*), added there 2026-06-15. When
  TC 153 gained its own `rate` parameter on 2026-08-08, the same validation pattern — including the
  `duration == 0` special case — was mirrored onto DOCKEDPROFILE for consistency, without re-examining
  whether "unbounded" belonged on a real flight command. It never should have: a docked profile must
  never be able to run forever. Fixed by rejecting `duration == 0` in the TC parser
  (`StrateoleXML@03fab5d`) and removing the `indefinite` code path from `Flight_DockedProfile` entirely
  — `NextPeriodLength` no longer takes an `indef` parameter, `ST_MEASURE_WAIT`'s remaining-time logic no
  longer branches on it, and the `MAX_DOCKED_PERIOD_TMS` guard no longer needs an unboundable-case
  carve-out.

## Design

### Structure: nest `Flight_PUOffload` inside `Flight_DockedProfile`

`Flight_PUOffload(true/false)` is called directly from the docked-profile state machine, the same way
[Flight_PUOffload.cpp](../src/Flight_PUOffload.cpp) already nests `Flight_CheckPU`. Each `Flight_*.cpp`
owns its file-static state, so nesting is safe, and the docked profile's own timing (local `millis()`
deadlines, see below) is disjoint from the offload's scheduler-based `RESEND_PU_RECORD`/`RESEND_TM`.

Deliberately **not** bouncing out through `FLM_IDLE` / `ACTION_OFFLOAD_PU` and re-entering: that would
lose the docked profile's static state, need a new "resume" action, and `FLM_IDLE`'s priority if/else
chain ([Flight.cpp](../src/Flight.cpp)) plus the 3-loop `FLAG_STALE` expiry could let a stray action
silently abandon the profile with the RPU parked in standby.

Consequence accepted: `inst_substate` stays `FLM_DOCKED` for the whole profile, so ground no longer sees
`FLM_PU_OFFLOAD` during a docked profile. The per-period text TM below replaces that visibility.

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
```

| State | Body | Next |
|---|---|---|
| `ST_ENTRY` | guard `docked_profile_rate != 0`; guard `docked_profile_time != 0` (defense-in-depth — the TC parser already rejects this); **reject if the longest period this profile will run needs too many RPU record blocks** (see below); `measure_remaining_s = docked_profile_time`; increment `profile_id`; capture `profile_start_lat/lon/alt` from `zephyrRX.zephyr_gps` | `ST_GO_MEASURE` |
| `ST_GO_MEASURE` | `period_len_s = NextPeriodLength(...)`; clear `pu_measure`/`pu_standby`; `TX_GoMeasure(period_len_s, docked_profile_rate, …)`; arm `pu_reply_deadline_ms` | `ST_CONFIRM_GO_MEASURE` |
| `ST_CONFIRM_GO_MEASURE` | on `pu_measure`: `period_deadline_ms = millis() + period_len_s*1000 + 2000` (2s head start so the RPU's own timer fires first). On timeout: retry once, else `SendTextTM("RPU not responding to go-measure command, returning to FLM_IDLE", WARN)` + `return true` | `ST_MEASURE_WAIT`, or done |
| `ST_MEASURE_WAIT` | When `period_deadline_ms` passes: `measure_remaining_s -= min(remaining, period_len_s)`; `resume_after_offload = (measure_remaining_s > 0)` | `ST_GO_STANDBY` |
| `ST_GO_STANDBY` | `SendTextTM(<period summary>, FINE)` (only on fresh entry, not a retry — see bug fix below); clear `pu_standby`; `TX_GoStandby(rpu_bat_temp)`; arm `pu_reply_deadline_ms` | `ST_CONFIRM_STANDBY` |
| `ST_CONFIRM_STANDBY` | on `pu_standby`: `docked_period_num++`; `Flight_PUOffload(true)`. On timeout: retry once, then **abort** (`SendTextTM("RPU did not confirm standby; aborting docked profile, returning to FLM_IDLE", WARN)` + `return true`) — see standby-abort note below | `ST_OFFLOAD`, or done |
| `ST_OFFLOAD` | `if (!Flight_PUOffload(false)) break;` then: offload failed → WARN + `return true` (abort); `!resume_after_offload` → `SendTextTM("Finished docked profile, returning to FLM_IDLE", FINE)` + `return true`; else | `ST_GO_MEASURE` |

Every exit hands control back to `FLM_IDLE`, and each announces itself:

| Exit | TM | Flag |
|---|---|---|
| `docked_profile_rate == 0` at entry | `Docked profile: invalid sample rate 0, returning to FLM_IDLE` | WARN |
| `docked_profile_time == 0` at entry | `Docked profile: invalid duration 0, returning to FLM_IDLE` | WARN |
| period too long for sample rate (block-count guard) | `Docked profile: period too long for sample rate, returning to FLM_IDLE` | WARN |
| go-measure unacked after 2 attempts | `RPU not responding to go-measure command, returning to FLM_IDLE` | WARN |
| go-standby unacked after 2 attempts | `RPU did not confirm standby; aborting docked profile, returning to FLM_IDLE` | WARN |
| offload failed | `Docked profile aborted: RPU offload failed, returning to FLM_IDLE` | WARN |
| all measurement time used | `Finished docked profile, returning to FLM_IDLE` | FINE |

Segment sizing, as a file-static helper so the rule is stated once:

```c
static uint16_t NextPeriodLength(uint16_t period, uint16_t rate, uint32_t remaining)
{
    if (period == 0) return (uint16_t)remaining;  // one period covers the whole profile
    if (period <= rate) period = rate + 1;
    if (remaining <= period) return (uint16_t)remaining;
    if ((remaining - period) <= rate) return (uint16_t)remaining; // fold a sub-sample tail in
    return period;
}
```

`remaining` is guaranteed nonzero on every call: `docked_profile_time` (its initial value) can't be 0
(rejected at `ST_ENTRY`), and `ST_OFFLOAD` only loops back to `ST_GO_MEASURE` when `measure_remaining_s >
0`. So this never returns 0.

Both clamps matter: the RPU silently rewrites a commanded duration when `duration <= rate`
(`RPU/src/RPU/RPU.cpp`), so an unclamped runt period would make RACHUTS and the RPU disagree about how
long that period ran and break the "periods sum to `docked_profile_time`" invariant.

### Timing: local `millis()` deadlines, not `scheduler.AddAction`

`Flight_DockedProfile` doesn't call `scheduler.AddAction` anywhere; all timing (`pu_reply_deadline_ms`,
`period_deadline_ms`) uses `millis()` deadlines compared with `(int32_t)(millis() - deadline) >= 0` (safe
across `millis()`'s ~49.7-day rollover). This closes [KnownIssues §15](KnownIssues.md) outright (a
silently-failed `AddAction` can no longer hang this file) and makes the file structurally immune to the
reentrancy bug demonstrated in [KnownIssues §13a](KnownIssues.md) (a stale scheduled action from an
earlier cancel-and-retry firing early into a new run) — there's nothing to leave stale, since nothing is
ever armed on the shared scheduler by this file. `ACTION_END_DOCKED_PROFILE` and `RESEND_PU_GOPROFILE`
are consequently unused by this file (left declared; `Flight_Profile.cpp` still uses the general pattern
for its own timers).

### New `ST_ENTRY` guard: bound record blocks per period (KnownIssues §3)

`Flight_PUOffload`'s `ST_WAIT_PACKET`/`ST_REQUEST_PACKET` arm `RESEND_TM`/`RESEND_PU_RECORD` per record
block without ever cancelling the previous one — a single offload needing too many blocks (each holding
`RPU_TM_MAX_RECORDS = 160` records) can fill the 32-slot scheduler queue on its own, independent of how
well periods are spaced from each other. `ST_ENTRY` now rejects the profile outright if the longest
period it would run needs more than `MAX_DOCKED_PERIOD_TMS` (`MAX_SCHEDULE_SIZE * 3/4` = 24) blocks:

```c
uint32_t longest_period_s = (0 == docked_offload_period) ? docked_profile_time : docked_offload_period;
if (longest_period_s > (uint32_t)MAX_DOCKED_PERIOD_TMS * RPU_TM_MAX_RECORDS * docked_profile_rate) {
    SendTextTM("Docked profile: period too long for sample rate, returning to FLM_IDLE", WARN);
    return true;
}
```

Runs after the `docked_profile_time != 0` guard, so `docked_profile_time` is always safe to read here.
Using it in place of the period when `docked_offload_period == 0` matters: that's legacy single-period
mode, where the *whole profile* is one offload burst, and checking the period value (`0`) instead would
trivially pass and miss exactly the case this guard exists for.

This prevents *this* call path from triggering the §3 overflow — it doesn't fix `Flight_PUOffload`'s own
uncancelled `AddAction` calls, which remain open for TC 147 manual offloads and any docked profile the
guard permits.

### Cancel (TC 156) routes into the shared path

The check stays at the top of the function, before the switch (KnownIssues §14), state-aware so a
cancel during an in-flight offload doesn't restart a second one:

```c
if (CheckAction(ACTION_CANCEL_MEASURE)) {
    resume_after_offload = false;      // never resume measure after a cancel
    measure_remaining_s = 0;
    if (profile_state == ST_OFFLOAD) {
        log_nominal("Cancel received during offload; finishing current offload");
    } else if (profile_state != ST_GO_STANDBY && profile_state != ST_CONFIRM_STANDBY) {
        SendTextTM("Docked profile cancelled", FINE);
        resend_attempted = false;      // fresh entry into the standby/offload sequence
        profile_state = ST_GO_STANDBY;
    }
    return false;                      // stay in FLM_DOCKED until the data is down
}
```

Idempotent, one code path for cancel/periodic/final. `TCHandler.cpp` still sends `TX_GoStandby`
unconditionally when TC 156 lands; the redundant one in `ST_GO_STANDBY` is harmless.

### Bugs found and fixed on the bench during the first build

**Retry-cap bug.** `resend_attempted` must not be reset inside `ST_GO_MEASURE`/`ST_GO_STANDBY`
themselves — only at true fresh-attempt entry points (`ST_ENTRY`, the confirm states' success paths, the
cancel handler's jump into `ST_GO_STANDBY`). Resetting it in the `GO_*` states erases the retry count the
confirm state had just set, so "retry once then give up" looped forever instead of ever reaching the
give-up branch. This pattern predated this rework (already present in the original go-measure retry
logic) and was initially carried into the new go-standby retry logic when it was added.

**Duplicate "segment complete" TM.** `ST_GO_STANDBY`'s summary TM is now gated on `!resend_attempted`, so
it fires once per period boundary regardless of how many times the underlying `TX_GoStandby` has to be
resent — previously a slow/lost standby ack duplicated the TM every retry.

### The standby-abort fix

`ST_CONFIRM_STANDBY` aborts the whole profile (one WARN) after two unacknowledged `TX_GoStandby`
attempts, at *any* period boundary — not just the final one. An earlier version of this logic let a
mid-profile failure "proceed into the offload anyway," reasoning the RPU accepts `RPU_SEND_RECORDS` from
any state, so an unconfirmed standby wasn't itself proof of a problem. Bench evidence showed that
reasoning wrong in practice:

```
13:17:50  RPU did not confirm standby; offloading anyway
13:18:11  PU not responding to status request           (2 retries)
13:18:32  PU not successful in sending profile record   (2 retries)
13:18:32  Docked profile aborted: RPU offload failed
```

Four WARN TMs and ~42s to reach the same conclusion — abort — that the direct path now reaches in ~12s
(2 × `RPU_RECEIVE_TIMEOUT`) with one message: if the RPU won't ack standby, the dock link is down, and
everything downstream in the offload attempt was always going to fail too.

*(An earlier version of this doc attributed this specific bench evidence to a `SerialComm::ReadChecksum()`
`available()`-vs-`peek()` divergence found while investigating a separate ground-TM blackout. That
divergence was real, but it was not the blackout's cause — the blackout was hardware, USB ground loops,
not a SerialComm bug. The `available()`/`peek()` fix built on that wrong diagnosis was reverted; the
current `SerialComm.cpp` uses a different, independently-tested `ReadChecksum` approach. So the actual
cause of the standby-ack loss above is unconfirmed — it may simply be ordinary dock-link unreliability.)*
The fix remains sound defense-in-depth regardless of cause: there's no scenario where "keep trying
anyway" beats a fast, clear abort for a genuine link/power failure.

## Files changed

**Firmware**
- [src/Flight_DockedProfile.cpp](../src/Flight_DockedProfile.cpp) — the rewrite above.
- [src/Flight_PUOffload.cpp](../src/Flight_PUOffload.cpp) — new `pu_offload_success`, set on all three
  `return true` paths (success, PU failure, unknown state), mirroring the existing `check_pu_success`
  convention in [Flight_CheckPU.cpp](../src/Flight_CheckPU.cpp).
- [src/PURouter.cpp](../src/PURouter.cpp) — `HandlePUAck`'s `RPU_GO_STANDBY` case sets `pu_standby = true`
  on ack and clears `pu_measure`; `RPU_GO_MEASURE` clears `pu_standby`.
- [src/StratoRachuts.h](../src/StratoRachuts.h) — `bool pu_standby`, `bool pu_offload_success`,
  `uint8_t docked_period_num` (a count/index, not a duration -- `docked_offload_period` is the
  configured period length); `RPU_TM_MAX_RECORDS` (160, duplicates the RPU-side constant) and
  `MAX_DOCKED_PERIOD_TMS` (`MAX_SCHEDULE_SIZE * 3/4`) for the new guard; `PU_RESEND_TIMEOUT` renamed to
  `RPU_RECEIVE_TIMEOUT` and retuned 10s → 6s (bench-measured RPU turnaround: ~0.6s status reply, ~1.1s
  go-measure/standby ack, ~1.9s largest 7692B record block — 6s keeps ~3x margin while halving
  worst-case dead time on an unresponsive RPU). Shared by `Flight_Profile.cpp`, `Flight_CheckPU.cpp`,
  `Flight_ReDock.cpp` — mechanical rename there, no other behavior change.
- [src/StratoRachuts.cpp](../src/StratoRachuts.cpp) — `SendRPUREPORT` StateDetails2 gains a `period:`
  field: `"profile:%u period:%u packet:%u records:%u"`.
- [src/Flight.cpp](../src/Flight.cpp) — the manual `ACTION_OFFLOAD_PU` (TC 147) path sets
  `docked_period_num = 0` before calling `Flight_PUOffload(true)`, so a standalone offload doesn't
  inherit a stale period number from an earlier docked profile.

**Design-error correction (2026-08-19)**: `StrateoleXML` `Telecommand.cpp`'s `DOCKEDPROFILE` case
(`03fab5d`, pushed upstream and refetched into the vendored copy) now rejects `dockedProfileTime == 0`
unconditionally, instead of special-casing it as "unbounded." `Flight_DockedProfile.cpp` gained a
matching `ST_ENTRY` guard (defense-in-depth) and dropped the `indefinite` variable, `NextPeriodLength`'s
`indef` parameter, and every branch that depended on either — see the Context section above.

**EEPROM config / TC 157** — unchanged by this rework; `docked_offload_period` (default 1800s) and
`SETDOCKEDOFFLOADPERIOD` were already in place, including TC 157's own rejection (not just a warning) of
`0 < period < ZEPHYR_RESEND_TIMEOUT` in `TCHandler.cpp`.

**Docs** — `docs/KnownIssues.md` §3 (new guard note), §13a (reentrancy risk closed for this file), §15
(closed), §17 (rewritten — the fire-and-forget gap this section described is now fixed), Appendix A
(`RPUREPORT` row updated for the `period:` field).

## Notes and known consequences

- **The RPU power-cycles its sensors at every boundary.** `enterStandby()` clears `sensorsEnabled` and
  calls `powerdownSensors()`, so OPC pump / TDLAS / TSEN / RS41 restart cold each period and the first
  records after a resume may be unsettled. Argues for a large default period (the 1800s default), not a
  small one.
- **`elapsed_s` restarts at 0 each period** — `enterMeasure()` resets `MeasureStartMillis`. Each block
  header carries a fresh `epoch_time`/`gps_lat`/`gps_lon`, the correct stitching key for ground
  processing.
- **`period == 0` no longer risks an unbounded RPU FIFO overflow** — `duration == 0` is rejected outright
  (see the design-error correction above), so `period == 0` (legacy single-period mode) now always has a
  finite `docked_profile_time` bound, and the `MAX_DOCKED_PERIOD_TMS` guard covers it.
- Capturing GPS at `ST_ENTRY` also fixes an existing bug: `SendRPUREPORT` reports
  `profile_start_lat/lon/alt`, which were only ever written by `PUStartProfile()` — a path the docked
  profile never took — so previously a docked-profile RPUREPORT carried the last manual profile's
  position, or zeros.

## Out of scope

Not touched: `Flight_PUOffload`'s own scheduler usage (KnownIssues §3 proper — still open for TC 147 and
for any docked profile the new guard permits), the RPU firmware, and the `FL_EXIT` gap where a
Zephyr-commanded mode switch leaves the RPU in MEASURE with nobody to stop it.

## Verification

1. `pio run -e rachuts` — compiles clean. **Done.**
2. Bench: `157,120` then `153,600,1` → 5 periods of 120s, each followed by a `RACHUTSTEXT` summary and
   `RPUREPORT`(s) with `period:` incrementing and `packet:` restarting; ends with "Finished docked
   profile, returning to FLM_IDLE".
3. `157,0` then a short profile → single period, identical to the original behavior.
4. Cancel (TC 156) mid-offload → offload finishes, no second one starts, no resume.
5. Disconnect/power off the RPU right at a period boundary → one WARN, ~12s, not the old 4-message/42s
   sequence.
6. Cancel-and-retry a docked profile (the §13a repro) → no early/false completion.
7. `157,0` then `153,5400,1` (rate=1s, single period ≈34 blocks, over the 24-block guard) → rejected with
   a WARN, profile never starts. Then `153,5400,5` (same duration, slower rate → ≈7 blocks) → accepted.
8. `153,0,10` → rejected at the TC layer (parser NAK, no `RACHUTSTCACK` "TC Docked Profile" success text)
   before it can even reach `Flight_DockedProfile`. Confirms duration == 0 no longer means "run forever."
