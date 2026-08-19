# RACHUTS / RPU — Known Bugs and Foibles

A running record of bugs, design weaknesses, and hardware quirks in the
RACHUTS PIB firmware and the RPU (Profiler Unit) firmware, with root causes and
status. Spans two repos: `StratoCore_RACHUTS` (PIB, the dock master) and `RPU`
(the profiler, the dock slave). Last updated 2026-08-19.

Status legend: **FIXED** · **MITIGATED** (worked around, root cause remains) ·
**OPEN** (identified, not yet addressed).

---

## 1. Dock-link record-offload "corruption" (RACHUTS ↔ RPU) — **RESOLVED**

**Symptom.** During a profile offload, `RPU_PROFILE_RECORD` blocks intermittently
fail their checksum on the RACHUTS side (`"Profile record checksum invalid
(len=...)"`). Failures are **probabilistic and scale with frame size**, not
deterministic by position. The framing stays intact (exact byte count,
terminator found, checksum digits parse).

**Root cause (found 2026-08-10): a `SerialComm::ReadChecksum()` race on the
closing semicolon — not link corruption.** Direct evidence: captured the exact
bytes handed to `TX_Bin()` on the RPU (`debug.tx`) and the exact bytes
`Read_Bin()` put in `binary_rx.bin_buffer` on RACHUTS (`debug.rx`) for a block
RACHUTS flagged `checksum invalid (len=7212)`. `cmp -l` found **zero byte
differences** — the payload RACHUTS received was byte-for-byte identical to
what the RPU sent. That falsifies "genuine bit errors during reception" as the
cause, at least for this (and very likely most) failures, and points squarely
at the checksum-comparison code itself. (Also ruled out: version skew between
the two repos' vendored `SerialComm`/`RPUComm` copies — diffed byte-identical.)

The actual bug: `ReadChecksum()`'s digit-reading loop
(`while (timeout > millis() && temp < 5)`) can exit at its 5-digit cap
*without ever having waited* for the checksum's closing `;` — every other
delimiter check in this file (`ReadSpecificChar`) busy-waits, bounded by
`timeout`, before reading its byte, but this one falls through to a single
**non-blocking** `if (';' != serial_stream->read()) return false;`. If that
byte hasn't physically landed in the UART buffer yet (plausible within a few
µs at the tail of a transmission), the read returns `-1`, and an otherwise
perfectly-received message is spuriously flagged checksum-invalid.

**Why this explains the frame-size correlation** (previously read as "longer
bursts → worse signal integrity"): the bug only fires when the *combined*
16-bit checksum (`(check_a << 8) | check_b`) needs all 5 decimal digits, which
happens whenever `check_a` (the upper byte) is roughly ≥ 40 — about **84% of
`check_a`'s possible range**. Whether `check_a` (a running `mod 256` sum of
every byte in the message) lands there depends on how much it's wrapped
around:
- **Short, low-byte messages** (ACKs, status requests — a handful of ASCII
  digits/delimiters) often have a raw byte sum that never even reaches 256, so
  `check_a` sits at that small literal sum, frequently well under the 5-digit
  threshold — the race rarely gets a chance to fire.
- **Long binary payloads** (profile-record blocks, thousands of bytes of
  varied sensor data) wrap `check_a` around 256 thousands of times over the
  message, effectively randomizing it across its full 0–255 range — so on any
  given transmission there's roughly an **84–86% chance** it lands in the
  5-digit zone.

So it isn't that long messages have more corruption opportunity — it's that
long, byte-varied messages are simply far more likely to *land on* the exact
checksum-digit-count that exposes the race, while short messages usually don't.
**This also means the same latent bug is present on every other checksummed
exchange in the system** (MCB link, Zephyr-adjacent ASCII/ACK traffic
wherever `SerialComm` is used) — it's just statistically much rarer to trigger
on short messages, which may explain some of the very infrequent, previously
unexplained aberrant behavior seen elsewhere and chalked up to noise.

**Fix applied.** The first attempt (2026-08-10, `SerialComm@0f355e1`) added a
bounded wait using `available()` before the closing-`;` read. That version was
superseded (2026-08-12, `SerialComm@d9e1ca6`, "Trying a different checksum
read logic") by a cleaner approach: rather than a separate trailing wait after
the digit loop, the closing `;` is now read inside the *same* `peek()`/`read()`
polling loop as the checksum digits, giving uniform semantics across the whole
checksum-plus-delimiter read instead of two differently-implemented steps back
to back:
```cpp
while (timeout > millis() && temp < 6) {
    read_ret = serial_stream->peek();
    if (-1 == read_ret) continue;
    rx_char = (char) read_ret;
    if (rx_char == ';') {
        serial_stream->read();
        found_delimiter = true;
        break;
    }
    rx_char = serial_stream->read();
    checksum_buffer[temp++] = rx_char;
}
if (!found_delimiter) return false;
```
Deliberately *not* implemented via `ReadSpecificChar`/`GetNextChar`, since
those update the running checksum accumulator — this trailing `;` must stay
outside the checksum (mirrors `WriteChecksum()`'s own trailing
`WriteChar(';')` on the TX side, written *after* `combined_checksum` is
already captured). This is the current, tested version of `ReadChecksum` -- unrelated to a
separate ground-TM blackout investigated around the same time, which turned
out to be hardware (USB ground loops), not a `SerialComm` bug. An intermediate
`ReadChecksum` revision was briefly (and incorrectly) suspected as the
blackout's cause; that revision has since been reverted and is not what's
described here.
**Upstreamed into the `SerialComm` GitHub source** (no longer just a local
vendored patch — a fresh `pio` lib fetch on either repo pulls this directly).

**Superseded theory (kept for history).** The original diagnosis blamed
physical-layer signal integrity on the RS-232 dock link — a **MAX3381ECUP**
charge-pump transceiver whose rails were thought to sag under a sustained
gap-free burst. **This is now believed wrong**: the dock link was since
switched from RS-232 to TTL (bypassing the MAX3381 transceiver entirely, as
the previous-generation PU used) and the failures persisted identically,
which is inconsistent with a transceiver-specific electrical cause and was
the observation that reopened this investigation. UART hardware RX error
flags (overrun/noise/framing/parity) were also directly instrumented on the
Teensy 4.1 LPUART peripheral and read **clean** across observed failures,
ruling out UART-peripheral-level corruption too. The contributing factors and
mitigations found under the old theory are kept below since they were real,
independently-useful fixes (or at least harmless), but their justification
("worst case for a marginal charge pump") should be considered superseded.

**Contributing factors found along the way (still valid fixes, regardless of
root-cause theory):**
- The RACHUTS `PU_SERIAL` RX ring buffer was **4096 bytes**, smaller than an
  8 KB record frame, so frames overflowed and were dropped before `Read_Bin`
  drained them. Fixed: `PU_SERIAL_BUFFER_SIZE` → **16384** (StratoRachuts.h).
- The RPU added an 8 KB `DOCK_SERIAL` TX buffer so `TX_Bin()` dumps a whole
  frame and the UART sends it gap-free for ~600 ms, instead of the legacy PU's
  paced small-TX-buffer output.

**Ruled out along the way (with evidence):**
- *Not* a software state bug — failures are random, not "second block."
- *Not* concurrent on-board activity — disabling the RPU TX buffer (forcing the
  loop to block through the burst, like the legacy PU) did **not** help.
- *Not* RAM2/DMAMEM corruption of the record buffer — moving `rpu_records` out
  of DMAMEM did **not** help.
- *Not* the Zephyr TM transmit between blocks — legacy multi-block offloads with
  a TM per block worked fine.
- *Not* a transceiver sleep/first-byte drop — a `\n` wake byte before
  `RPU_SEND_RECORDS` did **not** help.
- *Not* RS-232 signal integrity — TTL swap did **not** help (see above).
- *Not* UART peripheral-level overrun/noise/framing/parity errors — instrumented
  directly, read clean across observed failures (see above).
- *Not* version skew between the two repos' vendored `SerialComm`/`RPUComm`
  copies — diffed byte-identical.
- *Not* a length limitation in the checksum algorithm itself — `check_a`/
  `check_b` are `uint8_t` accumulators with intentional `mod 256` wraparound,
  mathematically length-agnostic; verified no buffer/loop-bound overflow at
  the sizes in use.

**Mitigations still in place (may be safe to relax once the fix above is
validated — kept for now as belt-and-suspenders):**
- RPU batch capped at **160 records** (`RPU_TM_MAX_RECORDS`, 7692 B/block —
  `RPU_BLOCK_HDR_BYTES(12) + 160 × RPU_RECORD_BYTES(48)`), leaving 500 bytes of
  margin against RACHUTS's `PU_BUFFER_SIZE` (8192, `StratoRachuts.h`). A
  `static_assert` was added in RPU.cpp (`RPU_TM_BUFFER_BYTES < 8092`, i.e. a
  required 100-byte headroom below 8192) so a future `RPURecord` growth that
  busts this margin now fails the RPU build instead of silently getting
  rejected at runtime on the RACHUTS side. Note this check is a hardcoded
  duplicate of RACHUTS's `PU_BUFFER_SIZE` value, not a shared constant — if
  `PU_BUFFER_SIZE` itself is ever changed on the RACHUTS side, this assert
  won't automatically track it and would need a matching update.
- RACHUTS now **accepts and forwards** a checksum-invalid `RPU_PROFILE_RECORD`
  to the ground rather than NAK/retrying (`PURouter.cpp::HandlePUBin`) — added
  2026-08-09, before the race was found, as a defensive measure against
  repeated-failure aborts. Worth reconsidering once the real fix is validated:
  if checksum failures become rare/nonexistent, reverting to NAK+retry (with
  the original all-or-nothing abort-after-two-failures behavior fixed
  separately) may be preferable to shipping known-corrupted data.

**Still open / to try.**
- Confirm on real flight hardware (validated via simulator so far).
- Also audit `Read_ASCII()`'s message-ID delimiter check (`SerialComm.cpp`,
  the `rx_char = serial_stream->peek(); if (rx_char != ',' && rx_char != ';')
  return false;` right after the id-digit loop) — same class of bug (a
  non-waiting check right after a loop that can exit at its own digit cap).
  Every equivalent check in `Read_Ack`/`Read_Bin`/`Read_String` already uses
  the safe, waiting `ReadSpecificChar` pattern; `Read_ASCII` is the one
  outlier. Not yet fixed.
- Once validated, reconsider whether the 160-record cap and
  accept-instead-of-retry mitigation are still necessary.

**Legacy comparison.** The legacy PU (`PUCode/RACHuTS_PU_V2_5.ino`) sent up to
200×30 B = 6000 B profile batches and 750×10 B = 7500 B TSEN batches reliably —
on *different PU hardware* and with a paced (small-TX-buffer) output. Given the
new root-cause theory, this is no longer necessarily evidence of a hardware
difference — a paced, small-buffer output would also change the checksum
value's digit-count distribution and message timing in ways that could have
simply made the legacy code less likely to hit this same race.

---

## 2. SerialComm protocol weaknesses (shared lib) — **OPEN (by design)**

The `SerialComm` protocol (`.pio/libdeps/.../SerialComm`) underpins the dock
(RPUComm) and MCB links. It is a mash-up of paradigms with no clean transport
guarantees:

- **No transport-level ack/nak/retransmit.** Checksums exist but are *advisory*
  flags (`checksum_valid`) the application must check. The ACK frame type is
  just an application-convention message.
- **Inconsistent checksum enforcement.** `HandlePUBin` checks `checksum_valid`;
  `HandlePUASCII` does **not**, so a corrupted-but-parseable command is acted on.
- **`RX()` gives up on the first failed parse.** A `"`/`!`/etc. mid-stream sends
  it into the wrong `Read_*` which fails, and the whole call returns
  `NO_MESSAGE` instead of resyncing — drains garbage only a few bytes per call.
- **No sequence/tag numbers**, so a duplicate or late reply is indistinguishable
  from a fresh one (CheckPU once processed two `RPU_STATUS` replies back-to-back).
- **Binary payloads can collide with frame delimiters**; resync depends on
  payload bytes not looking like delimiters.

Cleanup priorities if a window opens: resync in `RX()` after a failed parse;
enforce checksums centrally in the routers; add a sequence byte to
request/reply pairs.

---

## 3. Scheduler queue overflow on long offloads (RACHUTS) — **OPEN (fix proposed)**

**Symptom.** Near the end of a large profile offload (~block 27+),
`"Schedule queue full"` errors every block, and resend timers stop arming.

**Root cause.** The scheduler is a fixed **32-slot** queue (`MAX_SCHEDULE_SIZE`,
StratoScheduler.h). Each offload block schedules two timeout actions that are
**never cancelled on success**:
- `ST_REQUEST_PACKET` → `AddAction(RESEND_PU_RECORD, RPU_RECEIVE_TIMEOUT)` (6 s)
- `ST_WAIT_PACKET` → `AddAction(RESEND_TM, ZEPHYR_RESEND_TIMEOUT)` (60 s)

A scheduled item only leaves the queue when its timer *fires* (or on mode
switch); `AddAction` does not dedupe. At ~1.7 s/block, the 60 s `RESEND_TM`
items accumulate (~27 + ~6 pending `RESEND_PU_RECORD` = 33 > 32) and overflow
around block 27. Once full, `AddAction` fails so later blocks' resend timers
never arm, and stale fired flags can trigger spurious resends in unrelated
blocks.

Note the same latent leak exists in the other state machines (Flight_Profile,
ReDock, ManualMotion) — they just don't iterate fast enough to overflow.

**Identified proper fix (deferred — shared-lib change).** The scheduler is *not
designed for the same action to be on the queue more than once*: there is a
single flag per action (`action_flags[action]`), so multiple queued copies of
one action are inherently meaningless. The right fix is therefore to make
`StratoScheduler::AddAction` **dedupe** — remove any existing queued items for
that action before adding the new one, i.e. "reschedule" rather than "stack."
This:
- requires **no** state-machine changes,
- hard-bounds the queue at the number of distinct action types
  (RACHUTS `NUM_ACTIONS` ≈ 30 < the 32-slot limit), so overflow becomes
  impossible, and
- matches the one-flag-per-action design intent.

It is **deferred** because `StratoScheduler` is shared by every StratoCore
instrument (RATS, etc.), so changing `AddAction` semantics is cross-cutting and
must be validated across all instruments and landed in the StratoCore source of
truth (not just this repo's vendored `.pio/libdeps` copy).

**Contained stopgaps (no shared-lib impact), if the overflow must go away
sooner:**
- Replace the offload's two `AddAction` timeouts with local `millis()` deadline
  timers inside `Flight_PUOffload.cpp`.
- Or temporarily shorten `ZEPHYR_RESEND_TIMEOUT` to ~20 s (queue occupancy ≈
  timeout ÷ block period, so < ~30 s stays under 32 at the current block rate) —
  fragile and rate-dependent, treat as temporary.

**Contained mitigation applied for one call path (periodic docked-profile
offload).** `Flight_DockedProfile`'s `ST_ENTRY` now rejects a profile outright
(`WARN`, no measurement starts) if the longest period it would run needs more
than `MAX_DOCKED_PERIOD_TMS` (`MAX_SCHEDULE_SIZE * 3/4` = 24) record blocks at the
commanded sample rate:
```
longest_period_s > MAX_DOCKED_PERIOD_TMS * RPU_TM_MAX_RECORDS * docked_profile_rate
```
using `docked_profile_time` in place of the period when `docked_offload_period
== 0` (single-period/legacy mode). This prevents *that* call path from ever
starting an offload burst large enough to hit this section's overflow — it does
not touch `Flight_PUOffload`'s own uncancelled `AddAction` calls above, which
remain the root cause and are still fully open for TC 147 (`OFFLOADPUPROFILE`)
manual offloads and for any docked profile the guard permits. `docked_profile_time`
is always nonzero by the time this runs (a docked profile must have a finite
duration — see the design doc's "design-error correction"), so this guard has
no unboundable case to carve out.

---

## 4. Zephyr-link MAX3381 sleep / first-byte drop (RACHUTS) — **FIXED**

**Symptom.** After >30 s of Zephyr-link inactivity, the first byte of the next
transmission is dropped.

**Root cause.** The MAX3381 has a ~30 s inactivity auto-powerdown; the first
byte after wake is lost.

**Fix.** `ZephyrTXpoke(ZephyrTXMsgType_t)` writes a throwaway `'\n'` to
`ZEPHYR_SERIAL` before the real send, absorbing the dropped byte. All Zephyr
sends (TM, S, IMR, RA) route through it. Mirrors the RATS fix (RACHUTS adds RA,
which RATS lacks).

---

## 5. RPU measurement cadence and unused parameters — **OPEN**

- **Sample cadence drifts to ~1.04 s/record** instead of 1.0 s. `tickMeasure`
  uses `if (tick_timer < 1000) return; tick_timer = 0;` — resetting to 0
  discards the per-loop overshoot, so the period locks in at ~1.04 s and
  accumulates (120 records span ~125 s). `elapsed_s` (set from
  `(millis() - MeasureStartMillis)/1000`) honestly reports the drifted times, so
  this is **not** record loss. Fix: `tick_timer -= 1000;` to preserve phase.
- **`MeasureRate` is ignored.** Received in `RPU_GO_MEASURE` but `tickMeasure`
  hard-codes 1000 ms.
- **`MeasureDuration` is ignored.** The RPU never auto-returns to STANDBY; it
  runs MEASURE until commanded.

---

## 6. Stale StateDetails leak in TMs (RACHUTS) — **FIXED**

`zephyrTX` keeps `details1/2/3` as persistent `String` members, and `clearTm()`
resets only the TM payload buffer, not the detail strings. Builders that set only
some details left the rest stale: `SendRPUStatusTM` (set 1, 2) inherited a stale
StateDetails 3 (`"PU TM: <id>.<n>, ..."`) from the most recent `SendProfileTM`,
and `SendMCBTM`/`AddMCBTM`/`SendMCBEEPROM`/`SendPIBEEPROM` (set only detail 1)
carried stale 2 and 3.

**Fix.** Each TM builder now sets all three details explicitly, using `""` for
unused ones. The XMLWriter only emits a `StateMessN` tag when the detail string
is non-empty (XMLWriter_v5.cpp), so an empty detail is *omitted* from the TM
rather than emitted blank — no stale content, no empty tags. `SendProfileTM`
already set all three.

---

## 7. RPU records consumed before ACK (RPU) — **FIXED**

The offload's `sendRPURecords()` originally `pop()`ed records from the FIFO as it
built each batch, destroying them before the dock ACK. A NAK then lost the whole
batch (and advanced to the next records). Fixed with a held `tm_buf` /
`tm_pending_records` that only clears on ACK, so a NAK retransmits the same
batch.

---

## 8. Stray debug print on the dock UART (RPU) — **FIXED**

`RPUStatus.cpp` had a leftover `DOCK_SERIAL.println(json)` in the periodic
LoRa-status function. `DOCK_SERIAL` is `Serial1` — the *same* UART as the framed
RPUComm dock protocol — so it periodically dumped unframed JSON onto the dock
link. On RACHUTS this sent `SerialComm::RX()` chasing `"` characters inside the
JSON, draining the bogus backlog a few bytes per loop and blocking recognition of
real framed `RPU_STATUS` replies — the original cause of intermittent CheckPU
timeouts. Removed; the USB `Serial.println(json)` was kept.

---

## 9. Hardware notes — RS-232 transceivers (MAX3381ECUP)

**Charge-pump/rail-droop theory retracted (2026-08-13).** The last bullet
below ("Charge-pump capacitors") documented a hypothesis that a marginal
MAX3381 charge pump was causing intermittent bit errors on the dock link
under sustained bursts. **This is invalid — confirmed in the lab:** the dock
link was swapped from RS-232 to TTL (bypassing the MAX3381 transceiver
entirely) and back again, and the symptom it was meant to explain was
unaffected either way. See §1's own "Superseded theory" note — the actual
root cause of that symptom was a `SerialComm::ReadChecksum()` software race
producing false checksum-invalid flags on data that had in fact arrived
byte-for-byte correctly, since fixed. The capacitor-value guidance below is
kept only as a historical record of a dead-end investigation, not as
actionable hardware advice — it should not be cited as a cause or fix for any
future comms symptom.

- Both the dock and Zephyr links use **MAX3381ECUP** RS-232 transceivers with
  AutoShutdown Plus.
- **Mode pins:** `FORCEOFF=LOW` → full shutdown; `FORCEON=LOW, FORCEOFF=HIGH` →
  auto-powerdown (30 s sleep, drops first byte on wake); **`FORCEON=HIGH,
  FORCEOFF=HIGH` → forced always-on** (charge pump runs continuously).
- **RPU** drives both `RS232_FORCEON` and `RS232_FORCEOFF` HIGH → forced on, so
  the RPU side never sleeps.
- **RACHUTS** `FORCEON_232` (41) / `FORCEOFF_232` (42) are marked *"Unused on
  MonDo and Rev E"* and never driven — the dock/Zephyr transceiver mode is set by
  hardware strapping. Worth confirming on the schematic whether that side is
  strapped forced-on or left in auto-powerdown.
- ~~**Charge-pump capacitors (relevant to §1 droop):** typical MAX338xE
  operating circuit is **0.1 µF** on all pump/reservoir caps (C1, C2 flying;
  C3 V+, C4 V− reservoir; VCC bypass) for 3.0–5.5 V; low-ESR X7R/X5R. Confirm
  the exact value vs the datasheet for the board's VCC. Undersized/high-ESR/
  cracked **reservoir caps (C3/C4)** cause the rail droop that produces the
  intermittent bit errors; bumping C3/C4 (e.g. 0.22–0.47 µF) improves holdup
  under sustained bursts.~~ **(retracted — see note above; there is no rail
  droop and no bit-error mechanism here.)**

---

## 10. Minor / latent (RPU) — **OPEN**

- **OPC parser** logs `"OPC parse error: too many fields"` intermittently —
  memory-safe (no overflow), but indicates OPC serial line-framing trouble
  (merged/partial lines). On failure `readOPC` returns the previous (stale) OPC
  values into the record (`gotOPC` is computed but unused), and the `static
  String buf` it accumulates into can grow unbounded if a newline never arrives.
- **`RPURecord` JSON debug print** is gated behind the `d` console command
  (default off) to avoid per-tick USB blocking and `String` heap churn; the
  console status print interval defaults to 0 (off), settable with `c <s>`.

---

## 11. Flight-only TCs silently no-op in the flight error state (RACHUTS) — **OPEN (fix planned)**

The flight-only TCs (RETRYDOCK 142, GETPUSTATUS 143, MANUALPROFILE 146,
OFFLOADPUPROFILE 147, DOCKEDPROFILE 153) just `SetAction(...)`; the action is
only *consumed* in `ManualFlight`'s `FLM_IDLE`. `RequireFlightMode` only checks
`mode_code == "FL"`, which is still true when the instrument is parked in the
flight **error loop** (`FL_ERROR_LOOP`, substate 9). So in the error state these
TCs pass the guard, set an action that `FLM_IDLE` never runs, and silently expire
(`WatchFlags` clears it after 3 loops) — no warning.

**Symptom:** after a profile/MCB fault drops RACHUTS to `MODE_ERROR` →
`FL_ERROR_LOOP`, TC 143 "gets no response" and TC 147 "returns no records," while
`RACHUTSREPORT` TMs keep flowing (`SendPeriodicRACHUTSREPORT()` runs at the top of
`FlightMode` regardless of substate, so the periodic report continues in the error
loop). `SENDSTATE` (TC 203) reports `mode: 1, substate: 9` (this substate's
numbering was previously pinned to 14 for continuity with pre-refactor logs;
since unpinned — see `FlightModeControlFlow.md`).

**Recovery today:** `EXITERROR` (TC 201) → back to `FLM_IDLE`, then the TCs work.

**Planned fix:** add a `flight_error_state` member (set true in `FL_ERROR_LANDING`,
cleared in `FL_ENTRY`) and reject in `RequireFlightMode` with a `WARN`
(`"<cmd> ignored: in flight error state (send EXITERROR)"`) so the command warns
instead of vanishing.

---

## 12. No RACHUTSREPORT on boot / mode entry — first report delayed a full period — **RESOLVED**

`SendPeriodicRACHUTSREPORT()` gates on
`(millis() - last_rachutsreport_ms) >= rpu_status_rate * 1000`. At boot `last_rachutsreport_ms`
initializes to 0 and `millis()` also starts near 0, so the elapsed time doesn't
reach a full period until `rpu_status_rate` seconds after boot. With the default
rate (1800 s) the first `SB, SB` report wasn't sent until **~30 min after boot**;
`SB_ENTRY` didn't force one. In general no report was ever sent immediately on
entering a reporting mode — the first one always waited one full period.

**Fix:** each reporting mode's `ENTRY` substate (SB/FL/SA/LP) now sets
`force_rachutsreport = true`, so a report goes out on the first loop after entry
(`force_rachutsreport` is honored even when `rpu_status_rate == 0`). Every mode
transition now emits a fresh `RACHUTSREPORT` — confirming the new mode + current RPU
status — and the periodic timer proceeds from there.

---

## 13. No way to cancel a specific scheduled action — stale actions can survive a `MODE_ERROR` episode — **OPEN**

`inst_substate = MODE_ERROR` (253) is a mode-agnostic forced control transfer:
any code (`MCBRouter`, `PURouter`, or a sub-state-machine escaping itself) can
write it with no knowledge of which mode is running, because every mode's own
enum aliases its own error-entry substate to the same numeric value
(`FL_ERROR_LANDING = MODE_ERROR`, `SB_ERROR_LANDING = MODE_ERROR`, etc. — see
`FlightModeControlFlow.md`). Whatever the mode's own state machine was doing is
discarded the moment its `switch (inst_substate)` next evaluates and lands on
that case.

**The gap:** there is no mechanism to cancel a *specific* action that was
scheduled before the fault. `StratoScheduler` is a single shared, time-ordered
queue; `ActionHandler` sets one flag per **action type**
(`action_flags[action].flag_value`), not per scheduling call. Two independent
places can leak a stale action across an error episode:

1. **The scheduler queue itself** — a not-yet-due action sitting in
   `StratoScheduler`'s linked list.
2. **`action_flags[]`** — once an action's timer fires, `RunScheduler()` (which
   runs *before* `RunMCBRouter`/`RunPURouter`/`RunMode` in `loop()`) has already
   set the flag, independent of the queue. `WatchFlags()` only ages an unchecked
   flag out after `FLAG_STALE` (3) loops.

`scheduler.ClearSchedule()` empties queue (1) but **never touches (2)** — an
already-fired flag survives it untouched. And only `Flight.cpp`'s
`FL_ERROR_LANDING` calls `ClearSchedule()` at all; `Standby`, `Safety`,
`LowPower`, and `EndOfFlight`'s own `*_ERROR_LANDING` do not, so even
queue-level protection is absent there. `StratoCore::RunMode()`'s automatic
`ClearSchedule()` on mode transitions doesn't help either, since entering
`MODE_ERROR` is a *substate* change — `inst_mode` never changes during an error
episode, so that safety net never fires.

**Risk profile:** a genuine *mode* switch (FL→SB, FL→SA, ...) is actually better
protected than an in-mode error episode — `StratoCore::RunMode()` calls
`scheduler.ClearSchedule()` unconditionally on every `inst_mode != new_inst_mode`
transition, centrally in the base class, with no per-mode opt-in required. But
it's the same function, so it shares the identical blind spot: an
already-fired `action_flags[]` entry survives a mode switch untouched, same as
it survives `FL_ERROR_LANDING`. In practice this issue is concentrated within
`MODE_FLIGHT`: nearly all runtime is spent in FL, and it's where nearly all
reel-motion fault paths live (motion timeouts, MCB faults, RPU faults), so a
fault-and-recover episode almost always stays inside FL (`FL_ERROR_LOOP` →
`EXITERROR` → `FLM_IDLE`) rather than crossing an actual mode boundary. The
other modes are both lower-traffic and lower-fault-rate, so the missing
`ClearSchedule()` call in their own `*_ERROR_LANDING` cases is a real gap but a
narrower one in practice.

**Concrete failure mode:** a sub-machine (e.g. `Flight_Profile`) schedules a
far-future action (e.g. `ACTION_END_DWELL`). A fault lands `inst_substate` on
`FL_ERROR_LANDING` before the action is due. If it's still queued,
`ClearSchedule()` removes it — handled. But if the action's timer already fired
in the same or an earlier loop (queue → `ActionHandler` → flag set) before the
fault was detected, or fires later while parked in `FL_ERROR_LOOP` awaiting
`EXITERROR` (TC 201), the flag survives `ClearSchedule()` untouched. Once
recovery re-enters `FLM_IDLE` and starts a *new*, unrelated command, if that
command's sub-machine happens to `CheckAction()` the same action ID, it
consumes the stale flag and reacts to a trigger left over from the aborted
attempt.

**Proposed fix:** add a `ClearActionFlags()` (or fold into `ClearSchedule()`)
that resets all of `action_flags[]`, and call it from every mode's
`*_ERROR_LANDING` (and `*_SHUTDOWN_LANDING`), not just Flight's. A more complete
fix would give the scheduler per-instance cancellation (e.g. a token returned
from `AddAction`, or dedupe-on-schedule as already proposed in §3) rather than
the current all-or-nothing queue clear plus separate, unrelated flag array.

### 13a. Re-entering a sub-machine before its own prior scheduled actions fire duplicates them in the queue

A more directly reachable variant of the same root cause — no fault required,
just re-invoking the same command. `scheduler.AddAction()` → `SchedulePush()`
never checks whether an entry for that `action` type is already queued; it
unconditionally allocates and links a new node. So restarting a sub-machine
(`restart_state = true`) while an earlier run's own scheduled action is still
pending produces **two independent queue entries for the same action type**,
each counting down to its own fire time.

**Concrete trace, using `Flight_ReDock`** (the sub-machine that most visibly
front-loads scheduled actions, in `ST_ENTRY`):

```cpp
case ST_ENTRY:
    redock_state = ST_IDLE;
    SetAction(ACTION_REEL_OUT);
    scheduler.AddAction(ACTION_IN_NO_LW, 30);
    scheduler.AddAction(ACTION_CHECK_PU, 60);
```

1. TC 142 (`RETRYDOCK`) → `Flight_ReDock(true)` → `ACTION_IN_NO_LW`@+30s and
   `ACTION_CHECK_PU`@+60s go on the queue.
2. Redock finishes quickly (docked confirmed at, say, +10s) → returns `true` →
   back to `FLM_IDLE`. Nothing clears the queue on this ordinary, successful
   return — `ACTION_IN_NO_LW`/`ACTION_CHECK_PU` are still queued, due at their
   original +30s/+60s.
3. Another TC 142 arrives before those fire → `Flight_ReDock(true)` runs again;
   `ST_ENTRY` re-adds `ACTION_IN_NO_LW`@+30s and `ACTION_CHECK_PU`@+60s a
   **second time**.
4. The queue now holds two independent entries for each action, with different
   fire times. When the *first* run's stale entry eventually pops,
   `ActionHandler` sets that flag regardless of what the *second* run's state
   machine is currently doing — a premature or duplicate trigger indistinguishable
   from the legitimate one.

**This is easily reachable via `CANCELMOTION` (TC 11), and inconsistently so**
— the three motion sub-machines handle it differently:

```cpp
// Flight_ManualMotion.cpp / Flight_ReDock.cpp, ST_MONITOR_MOTION — identical:
if (CheckAction(ACTION_MOTION_STOP)) {
    SendTextTM("Commanded motion stop", FINE);
    return true;              // clean finish -- no scheduler touch at all
}

// Flight_Profile.cpp, ST_MONITOR_MOTION -- different:
if (CheckAction(ACTION_MOTION_STOP)) {
    SendTextTM("Commanded motion stop in autonomous", WARN);
    inst_substate = MODE_ERROR;   // escalates -> FL_ERROR_LANDING -> ClearSchedule()
    break;
}
```

`Flight_ManualMotion` and `Flight_ReDock` treat a cancel as a clean success —
no `MODE_ERROR`, so no `ClearSchedule()` anywhere in that path; anything
already scheduled and not yet fired stays queued. `Flight_Profile`'s cancel
happens to get the queue cleared only as a side effect of being classified as
an error (which routes through Flight's `FL_ERROR_LANDING`) — not from any
deliberate cleanup logic. So a cancel-and-retry on a manual motion or a redock
is a live, easily reproduced way to hit this; a cancel-and-retry on a profile
incidentally isn't (today), by accident rather than by design.

**Proposed fix:** same as above (§13) — either give `AddAction` dedupe
semantics (reschedule an existing entry for the same action type instead of
adding a second one), or make `ClearSchedule()`/`ClearActionFlags()` run
consistently on every sub-machine's clean return, not just on `MODE_ERROR`
paths.

**Observed in the wild: `Flight_DockedProfile` / `ACTION_END_PREPROFILE`
(2026-08-12 session).** A first `TC Docked Profile` (21:13:00, 21600 s) armed
`ACTION_END_PREPROFILE` for ≈03:13:03. `TC Cancel Measure` at 21:16:04
returned cleanly from `Flight_DockedProfile` without touching the still-queued
timer. A second `TC Docked Profile` at 21:16:39 re-entered `ST_MEASURE_WAIT`
and was sitting there when the first run's stale timer fired at 03:13:03,
3 m 36 s before the real end — `Flight_DockedProfile` read the leftover flag
as its own completion and started an offload early. The RPU itself was
unaffected (its onboard timer, set independently by the go-measure command,
completed correctly at 03:16:39.647) only because the go-standby sent on that
false completion (see §17) never reached it.

This also surfaced a second, independent problem: `ACTION_END_PREPROFILE` is
shared between two unrelated sub-machines. `Flight_Profile.cpp` uses it for
the genuine "pre-profile" instrument warmup wait (`pibConfigs.preprofile_time`,
default 180 s) ahead of a manual profile's reel-out; `Flight_DockedProfile.cpp`
reused the same value for the end of the entire multi-hour measurement. Since
`action_flags[]` is one flat, instrument-wide array keyed by action *type*,
this reuse widened the collision surface beyond "re-entering the same
sub-machine" — a stale, not-yet-fired `ACTION_END_PREPROFILE` left behind by
an aborted *manual* profile could in principle be misread as completion by an
unrelated docked profile (and vice versa). Fixed by giving
`Flight_DockedProfile` its own dedicated `ACTION_END_DOCKED_PROFILE` symbol;
the underlying reentrancy gap (a stale same-machine timer surviving a
cancel-and-retry) is unchanged and still needs the §13/§3 `AddAction` dedupe
fix, or a local expected-fire-time guard, to be fully closed.

**Closed for `Flight_DockedProfile` specifically (periodic-offload rebuild).**
The rebuilt `Flight_DockedProfile` no longer calls `scheduler.AddAction` at
all — `ACTION_END_DOCKED_PROFILE` and `RESEND_PU_GOPROFILE` are unused by this
file now, replaced by local `millis()` deadlines compared with
`(int32_t)(millis() - deadline) >= 0`. A cancel-and-retry can no longer leave a
stale scheduled action behind for this file to misread, since none is ever
armed. The general `AddAction`-never-dedupes root cause (§13/§3) is unchanged
and still open for every other sub-machine, including `Flight_PUOffload`,
which this rebuild nests inside `Flight_DockedProfile` without changing.

---

## 14. `CANCELMOTION` only cancels a profile during its motion-in-progress window — **PARTIALLY FIXED**

`CANCELMOTION` (TC 11) sets `ACTION_MOTION_STOP` unconditionally, but
`Flight_Profile` only calls `CheckAction(ACTION_MOTION_STOP)` in **one** of its
12 states — `ST_MONITOR_MOTION` (entered while a reel-out/reel-in/dock motion
is physically underway, after `ST_START_MOTION`/`ST_VERIFY_MOTION`). Every
other state ignores it: the RA handshake (`ST_SEND_RA`/`ST_WAIT_RAACK`), the PU
go-measure handshake (`ST_SET_PU_PROFILE`/`ST_CONFIRM_PU_PROFILE`), the
pre-profile wait (`ST_PREPROFILE_WAIT`), the dwell wait (`ST_DWELL`), the
dock-wait timer (`ST_DOCK_WAIT`), the post-dock PU check
(`ST_GET_PU_STATUS`/`ST_VERIFY_DOCK`/`ST_REDOCK`), and the final MCB-low-power
confirm (`ST_CONFIRM_MCB_LP`).

So `CANCELMOTION` only actually cancels a profile while the reel happens to be
mid-motion. Sent during any other phase (e.g. mid-dwell), the flag isn't read
there — it either goes stale after 3 loops (`WatchFlags`) and is silently
dropped, or **survives until the profile later reaches `ST_MONITOR_MOTION` for
a subsequent, unrelated motion leg** (e.g. the reel-in after dwell) and cancels
*that* instead — a stale, misdirected cancel with no relationship to what was
actually intended. Same family as §13/§13a: a flag set once, consumed
opportunistically by whichever state happens to poll for it, with no
per-request scoping.

Related, and the reason this was found: `Flight_DockedProfile` has **no** cancel
handling at all (see the "docked profile has no motion" discussion) — arguably
correct in spirit, since `ACTION_MOTION_STOP` is the wrong verb for stopping an
in-place RPU measurement, but the practical effect is that neither operation
can be reliably cancelled mid-flight from the ground.

**Fix (docked profile only):** added `CANCELMEASURE` (TC 156) — a dedicated,
correctly-named cancel, distinct from `CANCELMOTION` (TC 11), since
`ACTION_MOTION_STOP` is the wrong verb for stopping an in-place RPU
measurement. It unconditionally sends the RPU to standby (`TX_GoStandby`, same
as `RPUGOSTANDBY`) and sets a new `ACTION_CANCEL_MEASURE` flag.
`Flight_DockedProfile` checks that flag **once, at the top of the function,
ahead of the state switch** — deliberately not state-specific, to avoid
exactly this issue's own failure mode — so a cancel takes effect immediately
regardless of which phase (go-measure handshake or measure-wait) the profile
is in, sends the collected data to offload, and returns.

**Still open (manual profile):** `Flight_Profile`'s per-state gap is
unchanged — `CANCELMOTION` still only works during `ST_MONITOR_MOTION`.
`CANCELMEASURE` is **not** the right tool here and should not be wired in:
it's specifically an RPU-directed command (`TX_GoStandby`), and a manual
profile in progress isn't fundamentally an RPU-communication operation the way
a docked profile is — most of its states are driven by MCB motion or local
timers, not by waiting on the RPU. `CANCELMOTION`/`ACTION_MOTION_STOP` remains
the semantically correct mechanism for `Flight_Profile`; the actual fix is
extending the `CheckAction(ACTION_MOTION_STOP)` check to every waiting state
(the RA handshake, PU go-measure handshake, pre-profile wait, dwell wait,
dock-wait timer, post-dock PU check, MCB-low-power confirm), not just
`ST_MONITOR_MOTION`. Deferred to later.

---

## 15. `Flight_DockedProfile` could hang forever if a scheduler `AddAction` silently fails — **RESOLVED**

**Symptom.** None observed; theoretical, found by code inspection.

**Original root cause.** `Flight_DockedProfile()`'s only paths to `return true`,
besides the ground-triggered `ACTION_CANCEL_MEASURE` cancel, were two scheduled
timer actions: `RESEND_PU_GOPROFILE` (`ST_CONFIRM_GO_MEASURE` — retried once,
then bails with a WARN if the RPU never ACKs) and `ACTION_END_PREPROFILE`
(`ST_MEASURE_WAIT`). Both were armed via `scheduler.AddAction(...)`, whose
`bool` return — `false` when the scheduler's 32-slot queue is full, see §3 —
was never checked at either call site. If `AddAction` silently failed, the
corresponding `CheckAction` never fired, and the state machine would sit
indefinitely with no automatic recovery — only a ground-issued
`CANCELMEASURE` (TC 156) could get it out.

**Fix (periodic-offload rebuild).** `Flight_DockedProfile` no longer calls
`scheduler.AddAction` anywhere — all its timing is local `millis()` deadlines,
which have no failure mode tied to queue occupancy. This closes the gap
outright for this file, independent of whether the general `AddAction` dedupe
fix (§3) ever lands. See §13a for the related reentrancy fix from the same
rebuild.

---

## 16. Zephyr-link CRC check is dead code; a corrupted TC is silently dropped with no NAK (shared lib) — **OPEN**

**Symptom.** A TC sent from the ground can simply vanish — no `RACHUTSTCACK`,
no `RACHUTSTCACK` WARN, no `TCAck`, nothing at all — while an unrelated TC sent
moments later works normally. Found while diagnosing a real ground log: TC 181
(`RPUSTATUSPERIOD`, well-formed) got zero response; TC 180 sent ~4.7 s later got
a normal (327 ms) WARN response. The instrument's main loop runs a fixed 1 Hz
cadence (`Timer1.initialize(100000)` × `LOOP_TENTHS=10`,
`StratoCore_RACHUTS.cpp:14,30,64`) and drains the Zephyr RX queue every tick, so
a multi-second gap with total silence on one message and a fast, clean response
on the next rules out any queueing/overwrite explanation — the first message was
never routed at all.

**Root cause — two compounding gaps in `XMLReader` (`StrateoleXML`,
`.pio/libdeps/.../StrateoleXML/XMLReader_v5.cpp`), shared by every StratoCore
instrument's ground uplink parser:**

1. **The CRC check is commented out.** `ReadVerifyCRC()` computes both
   `read_crc` (from the message) and `crc_result` (the running computed CRC),
   but the actual comparison is dead: `return true;
   //((uint16_t) read_crc == crc_result);` (`XMLReader_v5.cpp:427`). So a
   bit-corrupted message that still happens to match every literal tag/field
   structure is accepted as if uncorrupted — the CRC field is transmitted and
   parsed but never actually checked.
2. **Any framing mismatch is a silent, unlogged drop.** `GetNewMessage()`
   (`XMLReader_v5.cpp:75-135`) returns `false` with no logging at the first
   failed stage — `MessageTypeOpen`, field parsing, `MessageTypeClose`,
   `ReadVerifyCRC`, or (for TCs) `ReadBinarySection()`'s byte-exact match of the
   literal 5-byte `"START"` sync marker and exact-length read
   (`XMLReader_v5.cpp:127,437-458`). A single dropped/flipped bit anywhere in
   that framing fails the match, and `RouteRXMessage()` — which is what sends
   the low-level `TCAck` — is never called (`StratoCore.cpp:118-128,155-161`).
   `MessageTypeOpen` resyncs on the next `<` (`XMLReader_v5.cpp:291-296`), so a
   corrupted message doesn't jam the link, it just disappears without a trace.

The Zephyr uplink shares the same MAX3381ECUP transceiver family already
implicated in the dock-link bit-error problem (§1, §9), so an isolated
single-message bit error here is entirely plausible — same physical-layer root
cause, different link, and here there's no application-level counter (§2 —
SerialComm's dock/MCB protocols share the "no transport ack/nak" weakness; the
Zephyr TC path is worse in one respect: it has a CRC field that *looks* like it
provides integrity checking but doesn't actually check it).

**Practical impact.** From the ground operator's perspective, "TC sent, no
response at all" is indistinguishable between "instrument never received it"
and "instrument is unresponsive" — there's no NAK to tell them which. The
existing `SendPeriodicRACHUTSREPORT`/heartbeat TMs are the only way to confirm
the instrument is alive; a silently-dropped TC otherwise looks identical to a
dead instrument until the operator notices the expected `RACHUTSTCACK` never
arrived and manually resends.

**Deferred (shared-lib change).** Like §2/§3, `XMLReader`/`XMLWriter` are
shared across all StratoCore instruments, so fixing this (enabling the real CRC
comparison in `ReadVerifyCRC`, and/or logging a `ZephyrLogWarn` on any
`GetNewMessage()` parse failure so a corrupted-but-framed-enough message at
least produces *some* TM) needs validating across instruments and landing in
the StratoCore source of truth, not just this repo's vendored copy.

---

## 17. `Flight_DockedProfile`'s end-of-profile go-standby is fire-and-forget — **RESOLVED**

**Original symptom.** `ST_MEASURE_WAIT`'s completion path called
`puComm.TX_GoStandby(...)` and immediately proceeded to
`SetAction(ACTION_OFFLOAD_PU)` with no ACK check and no resend armed — unlike
`ST_GO_MEASURE`/`ST_CONFIRM_GO_MEASURE`, which retried once via
`RESEND_PU_GOPROFILE` before giving up. `PURouter::HandlePUAck()` had a `case
RPU_GO_STANDBY:` handler that logged the ACK/NAK but set no flag, so there was
no signal `Flight_DockedProfile` could even check. Combined with the
SerialComm "no transport ack/nak" weakness (§2), a single lost/garbled
go-standby frame went completely unnoticed — confirmed on 2026-08-12 (see
§13a): during the premature-completion episode there, the RPU never received
the go-standby command and kept measuring, unaware it had been told to stop,
until its own onboard timer correctly ended the measurement 3 m 36 s later.

**Fix (periodic-offload rebuild).** `HandlePUAck()`'s `RPU_GO_STANDBY` case now
sets a new `pu_standby` flag on ack (and clears it on the `RPU_GO_MEASURE`
case, and vice versa, so the two edge-triggered flags can't cross-contaminate
between periods). Every standby transition in `Flight_DockedProfile` — end of
a measurement period, not just end of profile, now that offloads happen
periodically — goes through a dedicated `ST_CONFIRM_STANDBY` state using the
same retry pattern as go-measure: one retry on `RPU_RECEIVE_TIMEOUT`, then a
second unacknowledged attempt aborts the whole docked profile immediately with
one WARN, regardless of whether it's a mid-profile period boundary or the
final one. (An earlier version of this fix let a mid-profile failure "proceed
into the offload anyway," reasoning the RPU accepts `RPU_SEND_RECORDS` from
any state — bench evidence showed that costs ~42 s and 4 WARN TMs to reach the
same eventual failure that aborting reaches in ~12 s with one message, since
an unresponsive RPU means the offload cannot succeed either way; simplified to
abort unconditionally.) See the design doc for the full state table.

---

## 18. DOCKEDPROFILE (TC 153) allowed an unbounded duration — a dev-only semantic leaked onto a flight command — **FIXED**

**Symptom.** `docked_profile_time == 0` meant "run until commanded to STANDBY" — a docked profile could
be started with no time limit at all, relying entirely on the operator remembering to send TC 156
(`CANCELMEASURE`).

**Root cause: a validation pattern mirrored from a dev-only command onto a real flight command, without
re-examining whether it belonged there.** The `duration != 0 && duration <= rate` special-case originates
in `RPUGOMEASURE` (TC 185, `StrateoleXML` `Telecommand.h`), which is explicitly marked *"Development
testing only — not used in flight operations"*, added 2026-06-15. When `DOCKEDPROFILE` (TC 153) gained
its own `rate` parameter on 2026-08-08, the commit description says it "mirrors RPUGOMEASURE's existing
pattern... including the same validation" — the `duration == 0` special case came along with it. That was
never a deliberate decision that a docked profile should be unboundable; it was inherited by copying a
validation pattern for consistency. A later fix session found `duration == 0` was additionally *buggy* —
ending the profile after ~2 s instead of actually running indefinitely as the (now-corrected)
`TelecommandCribSheet.md` documented — and fixed it to work as documented, which is what surfaced the
question of whether "indefinite" should exist here at all.

**Fix.** `StrateoleXML` `Telecommand.cpp`'s `DOCKEDPROFILE` case (`03fab5d`) now rejects
`dockedProfileTime == 0` unconditionally — `duration` must always be nonzero and greater than `rate`.
`Flight_DockedProfile.cpp` gained a matching `ST_ENTRY` guard (defense-in-depth, in case
`docked_profile_time` is ever set some other way) and the `indefinite` code path was removed entirely:
`NextPeriodLength` no longer takes an `indef` parameter, `ST_MEASURE_WAIT`'s remaining-time logic no
longer branches on it, the periodic-offload block-count guard (§3) no longer needs an unboundable-case
carve-out, and the RPU-FIFO-overflow risk that combination could cause is now structurally impossible.
See [the design doc](DockedProfilePeriodicOffload.md) for the full before/after.

**`RPUGOMEASURE` itself is unaffected** — it's still dev-only and `duration == 0` still means
"unlimited" there, which is fine for its intended use.

---

## Appendix A — RACHUTS Telemetry (TM) catalog

Every RACHUTS TM is a Zephyr/StrateoleXML telemetry message identified on the
ground by its **StateMess1** tag, optionally with **StateMess2/StateMess3**
detail strings and **StateFlag1–3** values, plus a binary payload added via
`zephyrTX.addTm(...)`. All are transmitted through `ZephyrTXpoke(ZEPHYRTX_TM)`
(wake byte + `zephyrTX.TM()`). Unless noted, StateFlag2/3 = `NOMESS` and
StateMess2/3 are empty (omitted from the XML).

| TM (StateMess1) | Builder | StateMess2 | StateMess3 | Flag1 | Binary payload |
|---|---|---|---|---|---|
| `RACHUTSREPORT` | `SendRACHUTSREPORT(rpu_block, source)` — sole caller is `SendPeriodicRACHUTSREPORT()` (see below) | `<mode>, <source>` — current RACHUTS mode code (`SB`/`FL`/`LP`/`SA`/`EF`) + source: block origin (`LORA` / `DOCK`) when an `rpu` block is present, or the mode code (e.g. `SB, SB`) on a header-only report | `Reel: <reel_pos>` (last-known reel position; refreshed only by MCB motion TMs) | `FINE` | JSON object, **variable length**: `{"rachuts":{"epoch","mode","substate","reel","src","rpu_age_s"}, "rpu":{...}}`. `epoch` is the PIB system time (Unix seconds via `now()`, like RATSREPORT's header epoch; unset until the RTC is set from GPS). The `rachuts` header is always present; the `rpu` block (from `RPUPacket::toJSON()` or the dock `RPU_STATUS` reply) is included **only when RPU status is available**, else absent. `rpu_age_s` = seconds since the last RPU status was received (`-1` if never). Ground must read `msg["rpu"]` and handle its absence; length is not fixed — don't hard-code it. |
| `RPUREPORT` | `SendRPUREPORT(packet_num)` (`StratoRachuts.cpp`; binary payload added earlier in `HandlePUBin`, PURouter) | `profile:<profile_id> period:<docked_period_num> packet:<packet_num> records: <n>` (`profile_id` is a RACHUTS-side EEPROM counter, incremented once per docked profile — not per period — not part of the RPU record itself; `docked_period_num` counts measure-then-offload periods within one docked profile, 0 for a standalone TC 147 offload, so `(profile_id, docked_period_num, packet_num)` stays unique across a multi-period profile) | `<pu_last_status>, <lat>, <lon>, <alt>` (or `PU Profile Record: unable to add status info`) | `FINE` (`WARN` if StateMess3 fails to format) | Binary `RPURecord` block — n × 48 B (`RPU_RECORD_BYTES`), capped at 160 records (`RPU_TM_MAX_RECORDS`) ≈ 7692 B/block. |
| `MCB TM Packet <n>` | `AddMCBTM()`, real-time mode | — | — | `FINE` | One MCB motion data packet, 29 B (`MOTION_TM_SIZE`). |
| `MCBACK` / `MCBASCII` / `MCBREPORT` / `MCBSTRING` | `SendMCBTM(TMname, flag, message)` (RATS-style) | the message (`message`), e.g. `MCB acked deploy acc`, `Finished profile reel out`, `MCB Fault: ...`, `MCBString: <err>` | `Reel: <reel_pos>` (current reel position) | `flag` (`FINE`/`CRIT`) | Accumulated `MCB_TM_buffer`. Non-real-time framing: 4-B start-epoch header (set in `NoteProfileStart`), then per packet `0xA5` sync + 2-B elapsed-tenths + 29-B motion data. |
| `MCB EEPROM Contents` | `SendMCBEEPROM()` | — | — | `FINE` | Raw MCB EEPROM dump (`mcbComm.binary_rx.bin_buffer`, `bin_length` B). |
| `RACHUTSEEPROM` | `SendPIBEEPROM()` | — | — | `FINE` | PIB/RACHUTS EEPROM dump (`pibConfigs.Bufferize` into the MCB binary RX buffer, `bin_length` B). |
| `RACHUTSTCACK` | `TCHandler()` (post-switch, RATS-style) | command summary (`msg2`), e.g. `Set dock_amount: 5.00`, `Sent go-measure to RPU: duration=130 rate=1` | detail/error (`msg3`), e.g. `Switch to manual mode before commanding motion` (empty on success) | `msg1_flag`: `FINE` ok / `WARN` rejected-or-error / `CRIT` unknown TC | none — sent once per received telecommand as the instrument-level ack. |

**`SendMCBTM` tags** (StateMess1) and where they come from:
- `MCBACK` — MCB command acks (config/limit acks, low power) from `HandleMCBAck`.
- `MCBASCII` — MCB motion faults / dock-detect from `HandleMCBASCII`.
- `MCBREPORT` — motion status from the flight state machines (`Finished profile
  reel out/in`, `Finished commanded manual motion`, `MCB Motion took longer than
  expected` (CRIT), `Unknown motion finished in profile monitor` (CRIT)).
- `MCBSTRING` — MCB error strings from `HandleMCBString` (`MCBString: <err>`,
  CRIT); also sets `MODE_ERROR`.

The message text is in StateMess2 and the current reel position in StateMess3.
`MCB_MOTION_FINISHED` and the reel-motion-start acks stay internal logs (the
state machine emits the `MCBREPORT`).

**Resends (not distinct TM types):** `Flight_ManualMotion` (ST after a motion TM)
and `Flight_PUOffload` (`ST_TM_ACK`) call `ZephyrTXpoke(ZEPHYRTX_TM)` to
re-transmit the **most recently built** TM from the XMLWriter on a NAK/timeout —
no new message is constructed.

**`RACHUTSREPORT` reporting model** (mode loops are the single sender):
- **Reception captures, it does not send.** LoRa (`LoRaRX()` in `InstrumentLoop`)
  and dock (`RPU_STATUS` in PURouter) store the decoded status into
  `latest_rpu_json` / `latest_rpu_src` and set `rpu_status_pending`. Sending a TM
  directly from `InstrumentLoop` raced the mode-loop TM and dropped LoRa reports.
- **`SendPeriodicRACHUTSREPORT()` is the only sender**, called every loop at the top
  of `StandbyMode`/`FlightMode`/`SafetyMode`/`LowPowerMode` (not EF). It transmits
  once per `rpu_status_rate` seconds (TC 181; `0` disables periodic), incorporating
  the most recent captured status if `rpu_status_pending`, else header-only.
  `millis()` math is unsigned (rollover-safe); emitting a block clears the pending
  flag so a status is reported once.
- **`force_rachutsreport`** lets a substate request an immediate report without doing
  the send itself (so time-critical substate work isn't blocked): `FLM_CHECK_PU`
  (TC 143) and `FLM_REDOCK` set the flag and the next mode loop transmits. Forced
  reports send even when `rpu_status_rate == 0`.

**Notes:**
- The `RACHUTSREPORT` builder (`SendRACHUTSREPORT`) was previously `RPUSTATUS` /
  `SendRPUSTATUS` (and originally `SendRPUStatusTM`); renamed to align with the
  RATS `RATSREPORT` convention (cf. `RACHUTSTEXT` / `RACHUTSTCACK` /
  `RACHUTSEEPROM`). `RPUREPORT` (`SendRPUREPORT`, formerly `SendProfileTM`) is the
  separate binary profile-record TM. Each builder's name matches its StateMess1 tag.
- The `RPUREPORT` payload is added to the TM buffer in `HandlePUBin` (PURouter)
  when the record block passes checksum; `SendRPUREPORT` only sets the state
  details/flags and transmits.
