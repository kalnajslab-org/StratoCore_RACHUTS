# RACHUTS / RPU — Known Bugs and Foibles

A running record of bugs, design weaknesses, and hardware quirks in the
RACHUTS PIB firmware and the RPU (Profiler Unit) firmware, with root causes and
status. Spans two repos: `StratoCore_RACHUTS` (PIB, the dock master) and `RPU`
(the profiler, the dock slave). Last updated 2026-06-13.

Status legend: **FIXED** · **MITIGATED** (worked around, root cause remains) ·
**OPEN** (identified, not yet addressed).

---

## 1. Dock-link record-offload corruption (RACHUTS ↔ RPU) — **MITIGATED**

**Symptom.** During a profile offload, `RPU_PROFILE_RECORD` blocks intermittently
fail their checksum on the RACHUTS side (`"Profile record checksum invalid
(len=...)"`). Failures are **probabilistic (~45% per 5700-byte block) and scale
with frame size**, not deterministic by position. The framing stays intact
(exact byte count, terminator found, checksum digits parse) — only payload
*values* are wrong, i.e. genuine bit errors during reception.

**Root cause.** Physical-layer signal integrity on the RS-232 dock link, error
rate ~1×10⁻⁴/byte. The link is RS-232 through a **MAX3381ECUP** charge-pump
transceiver on each end; the rails sag under a sustained gap-free burst, flipping
bits. The earlier "first block works, later blocks fail" and "fails above N
records" impressions were small-sample artifacts of this probability curve.

**Contributing factors found along the way:**
- The RACHUTS `PU_SERIAL` RX ring buffer was **4096 bytes**, smaller than an
  8 KB record frame, so frames overflowed and were dropped before `Read_Bin`
  drained them. Fixed: `PU_SERIAL_BUFFER_SIZE` → **16384** (StratoRatchuts.h).
- The RPU added an 8 KB `DOCK_SERIAL` TX buffer so `TX_Bin()` dumps a whole
  frame and the UART sends it **gap-free** for ~600 ms. This *unmasked* the SI
  problem (a saturated burst is the worst case for a marginal charge pump);
  the legacy PU's small TX buffer paced the output and never exposed it.

**Ruled out (with evidence):**
- *Not* a software state bug — failures are random, not "second block."
- *Not* concurrent on-board activity — disabling the RPU TX buffer (forcing the
  loop to block through the burst, like the legacy PU) did **not** help.
- *Not* RAM2/DMAMEM corruption of the record buffer — moving `rpu_records` out
  of DMAMEM did **not** help (and a bad RAM read would checksum-consistently,
  yielding valid-checksum-wrong-data, not a mismatch).
- *Not* the Zephyr TM transmit between blocks — legacy multi-block offloads with
  a TM per block worked fine.
- *Not* a transceiver sleep/first-byte drop — a `\n` wake byte before
  `RPU_SEND_RECORDS` did **not** help (this is mid-burst corruption, not a
  first-byte-after-idle drop).

**Mitigations in place.**
- RPU batch capped at **120 records** (~4560 bytes, `RPU_TM_MAX_RECORDS`) — lower
  per-frame failure probability than 150/200 (200 failed almost always). The
  ~45% figure above was measured at the earlier 150-record/5700-byte cap.
- Per-block NAK retransmit on the RPU + RACHUTS `RESEND_PU_RECORD` timeout.

**Still open / to try.**
- The link is genuinely marginal; retransmits recover single failures but the
  offload still aborts when two land back-to-back. The real fix is hardware:
  **inspect the MAX3381 charge-pump capacitors** (see §9) and/or **lower the
  dock baud** (115200 → 57600) to buy timing margin — note the `Read_Bin`
  timeout is 1000 ms, so watch the block transfer time if you drop baud or raise
  the batch size (the current ~4560-byte block is ~790 ms at 57600; a larger
  block or lower baud would brush the limit).

**Legacy comparison.** The legacy PU (`PUCode/RACHuTS_PU_V2_5.ino`) sent up to
200×30 B = 6000 B profile batches and 750×10 B = 7500 B TSEN batches reliably —
on *different PU hardware* (different transceiver/charge pump) and with a paced
(small-TX-buffer) output. So "it worked before" does not transfer to the new RPU
board.

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
- `ST_REQUEST_PACKET` → `AddAction(RESEND_PU_RECORD, 10 s)`
- `ST_WAIT_PACKET` → `AddAction(RESEND_TM, 60 s)`

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
- **Charge-pump capacitors (relevant to §1 droop):** typical MAX338xE operating
  circuit is **0.1 µF** on all pump/reservoir caps (C1, C2 flying; C3 V+, C4 V−
  reservoir; VCC bypass) for 3.0–5.5 V; low-ESR X7R/X5R. Confirm the exact value
  vs the datasheet for the board's VCC. Undersized/high-ESR/cracked **reservoir
  caps (C3/C4)** cause the rail droop that produces the intermittent bit errors;
  bumping C3/C4 (e.g. 0.22–0.47 µF) improves holdup under sustained bursts.

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

## Appendix A — RACHUTS Telemetry (TM) catalog

Every RACHUTS TM is a Zephyr/StrateoleXML telemetry message identified on the
ground by its **StateMess1** tag, optionally with **StateMess2/StateMess3**
detail strings and **StateFlag1–3** values, plus a binary payload added via
`zephyrTX.addTm(...)`. All are transmitted through `ZephyrTXpoke(ZEPHYRTX_TM)`
(wake byte + `zephyrTX.TM()`). Unless noted, StateFlag2/3 = `NOMESS` and
StateMess2/3 are empty (omitted from the XML).

| TM (StateMess1) | Builder | StateMess2 | StateMess3 | Flag1 | Binary payload |
|---|---|---|---|---|---|
| `RPUSTATUS` | `SendRPUSTATUS(json, source)` | `<mode>, <source>` — current RACHUTS mode code (`SB`/`FL`/`LP`/`SA`/`EF`) + source (`LORA` / `FLM_CHECK_PU` / `FLM_REDOCK`), e.g. `FL, LORA` | — | `FINE` | RPU status as a JSON text string, **variable length** (received into a 512-byte buffer on RACHUTS). From `RPUPacket::toJSON()` (LoRa path) or the dock `RPU_STATUS` reply. The `RPUPacket` field set changes over time, so the length is not fixed — don't hard-code it. |
| `RPUREPORT` | `SendRPUREPORT(packet_num)` (payload added in `HandlePUBin`, PURouter) | `Number of RPURecords: <n>` | `PU TM: <profile_id>.<packet_num>, <pu_last_status>, <lat>, <lon>, <alt>` (or `PU Profile Record: unable to add status info`) | `FINE` (`WARN` if StateMess3 fails to format) | Binary `RPURecord` block — n × 38 B (`RPU_RECORD_BYTES`), capped at 120 records (`RPU_TM_MAX_RECORDS`) ≈ 4560 B/block. |
| `MCB TM Packet <n>` | `AddMCBTM()`, real-time mode | — | — | `FINE` | One MCB motion data packet, 29 B (`MOTION_TM_SIZE`). |
| _motion message_ (see below) | `SendMCBTM(flag, message)` | — | — | `flag` (`FINE`/`CRIT`) | Accumulated `MCB_TM_buffer`. Non-real-time framing: 4-B start-epoch header (set in `NoteProfileStart`), then per packet `0xA5` sync + 2-B elapsed-tenths + 29-B motion data. |
| `MCB EEPROM Contents` | `SendMCBEEPROM()` | — | — | `FINE` | Raw MCB EEPROM dump (`mcbComm.binary_rx.bin_buffer`, `bin_length` B). |
| `RATCHUTSEEPROM` | `SendPIBEEPROM()` | — | — | `FINE` | PIB/RACHUTS EEPROM dump (`pibConfigs.Bufferize` into the MCB binary RX buffer, `bin_length` B). |

**`SendMCBTM` messages** (the StateMess1 value, set by the caller): `Finished
profile reel out`, `Finished profile reel in`, `Finished commanded manual
motion`, `MCB Motion took longer than expected` (CRIT), `Unknown motion finished
in profile monitor` (CRIT), `MCB dock detected: ...`, `MCB Fault: ...`, and the
MCBRouter-built `log_array` strings.

**Resends (not distinct TM types):** `Flight_ManualMotion` (ST after a motion TM)
and `Flight_PUOffload` (`ST_TM_ACK`) call `ZephyrTXpoke(ZEPHYRTX_TM)` to
re-transmit the **most recently built** TM from the XMLWriter on a NAK/timeout —
no new message is constructed.

**Notes:**
- `RPUSTATUS` and `RPUREPORT` builders were renamed from `SendRPUStatusTM` /
  `SendProfileTM` so the function name matches the StateMess1 tag.
- The `RPUREPORT` payload is added to the TM buffer in `HandlePUBin` (PURouter)
  when the record block passes checksum; `SendRPUREPORT` only sets the state
  details/flags and transmits.
