# RACHUTS Telemetry (TM) Crib Sheet

Reference for the telemetry messages (TMs) RACHUTS (PIB) sends to the ground,
and how the Zephyr protocol's StateFlag/StateMess slots are used to carry
them. Source of truth: `StrateoleXML/XMLWriter_v5.h/.cpp` (protocol),
`StratoCore/StratoCore.cpp` (base class), and `src/StratoRachuts.cpp`,
`src/TCHandler.cpp`, `src/MCBRouter.cpp`, `src/PURouter.cpp`, and the
`src/Flight_*.cpp` state files (senders). Last updated 2026-08-19.

---

## The StateFlag / StateMess protocol

Every Zephyr TM carries **3 flag/message slots**: `StateFlag1/2/3` +
`StateMess1/2/3`, set via `zephyrTX.setStateFlagValue(n, StateFlag_t)` and
`zephyrTX.setStateDetails(n, string)` before the TM is sent with
`zephyrTX.TM()` (binary/JSON payload) or `zephyrTX.TM_String()` (bare text,
used only by the base class's `ZephyrLogFine/Warn/Crit`).

`StateFlag_t` (`XMLWriter_v5.h`): `UNKN`, `FINE`, `WARN`, `CRIT`, `NOMESS` —
ground-station severity levels. `NOMESS` suppresses that slot's flag/message
from the XML entirely (used for slots 2/3 when unused, e.g. on pure binary
dump TMs).

RACHUTS has no dedicated "TM type" tag in the protocol — it identifies each
TM's type purely by convention, using the 3 slots as follows:

- **Slot 1** — identity + severity: `StateMess1` holds the TM name string
  (`"RACHUTSREPORT"`, `"MCBACK"`, `"RPUREPORT"`, etc.), and `StateFlag1`
  carries that TM's actual severity (the caller-supplied `state_flag`/`flag`
  argument). This is the only slot that varies in severity.
- **Slot 2** — human-readable context: mode+source for RACHUTSREPORT, the log
  text for RACHUTSTEXT/MCB* messages, profile/period/packet counters for
  RPUREPORT. Always forced `FINE` — slot 2 never carries its own severity.
- **Slot 3** — fixed-format side channel: reel position (`"Reel: X.XX"`) on
  nearly every motion-relevant TM, or `pu_last_status`/lat/lon/alt on
  RPUREPORT. Also always forced `FINE`.
- Pure binary/dump TMs with no narrative (MCBEEPROM, RACHUTSEEPROM) set
  slots 2/3 to `NOMESS` with empty details — only slot 1's `FINE` + name is
  meaningful.

Every TM send is wrapped by `ZephyrTXpoke(ZEPHYRTX_TM)`, which writes a
throwaway byte to `ZEPHYR_SERIAL` first to wake the MAX3381 transceiver
(it powers down after 30s of inactivity and can drop the first sent byte).

### TM delivery ack (`TM_ack_flag`)

`TM_ack_flag` (`ACK`/`NAK`/`NO_ACK`, base `StratoCore` enum) tracks the
Zephyr ground link's acknowledgment of the *last sent TM*, set in
`RouteRXMessage()` from the incoming `TMAck` Zephyr message. Most RACHUTS TM
senders explicitly force it to `NO_ACK` immediately after sending
(fire-and-forget). The two exceptions that actually wait on it:

- `Flight_ManualMotion.cpp` — after sending an `MCBREPORT`, waits for
  `ACK == TM_ack_flag`; retries via the `RESEND_TM` scheduled action on
  `NAK` or timeout.
- `Flight_PUOffload.cpp` — same pattern after sending an `RPUREPORT`.

These are the only two places TM delivery is actually confirmed rather than
assumed.

---

## TM table

| TM name (StateMess1) | Sender | Payload | When sent |
|---|---|---|---|
| `RACHUTSREPORT` | `SendRACHUTSREPORT` (`StratoRachuts.cpp`) | JSON: `{"rachuts":{...}}` header, optional `"rpu":{...}` block | Every mode loop (SB/FL/SA/LP) via `SendPeriodicRACHUTSREPORT`, on the configured `rpu_status_rate` period or immediately when `force_rachutsreport` is set (e.g. TC 143 GETPUSTATUS) |
| `RACHUTSTEXT` | `SendTextTM` (`StratoRachuts.cpp`) | none (StateMess2 = message) | RACHUTS's general-purpose event/error log — called from nearly every flight state file for warnings, aborts, and confirmations |
| `RACHUTSTCACK` | `TCHandler.cpp` | none | After every telecommand is processed (ack/nak summary) |
| `MCBREPORT` | `SendMCBTM` (`StratoRachuts.cpp`) | binary `MCB_TM_buffer` (accumulated motion telemetry) | End of an MCB motion (reel out/in, manual motion, dwell) — success or timeout |
| `MCBASCII` | `SendMCBTM` | binary MCB TM buffer | MCB ASCII messages relayed up (dock detection, fault info) |
| `MCBACK` | `SendMCBTM` | binary MCB TM buffer | Each MCB command ack forwarded (low power, cancel motion, limits set, zero reel, etc.) |
| `MCBSTRING` | `SendMCBTM` | binary MCB TM buffer | Free-text string messages from MCB |
| `MCBEEPROM` | `SendMCBEEPROM` (`StratoRachuts.cpp`) | binary EEPROM dump | On TC 18 (GETMCBEEPROM), once MCB EEPROM contents arrive |
| `RACHUTSEEPROM` | `SendPIBEEPROM` (`StratoRachuts.cpp`) | binary PIB/RACHUTS EEPROM (`pibConfigs`) dump | Deferred action after a TC 152 (GETPIBEEPROM) ack |
| `RPUREPORT` | `SendRPUREPORT` (`StratoRachuts.cpp`) | binary RPU profile record block | Once per record block during a PU offload (manual TC 147, or nested inside a docked profile's periodic offload) |
| *(unnamed, bare)* | Base class `ZephyrLogFine/Warn/Crit` via `zephyrTX.TM_String()` | none | Only fires from base `StratoCore.cpp` internals (e.g. "Zephyr comm loss timeout", watchdog reset) — RACHUTS itself never calls these directly, it always goes through `SendTextTM`/`RACHUTSTEXT` instead |
| `TM buffer as requested` | Base class `SendTMBuffer()` | full buffered TM contents | TC 202 (GETTMBUFFER), implemented in `StratoCore`, not overridden here |

---

## RACHUTSTEXT message inventory

`RACHUTSTEXT` (via `SendTextTM`) is RACHUTS's catch-all event/error log, so
its actual content is scattered across every flight state file as literal
strings rather than being enumerable from a single enum. This table is a
snapshot of every `SendTextTM(...)` call site as of 2026-08-19 — it will go
stale if messages are edited or added without updating this doc, so treat
wording as indicative, and use `grep -rn SendTextTM src/` to check current
truth. Not included: the analogous free-text messages carried inside
`MCBASCII`/`MCBACK`/`MCBSTRING` TMs, since that wording is the MCB system's
domain, not RACHUTS's.

This inventory exists to support standardizing RACHUTSTEXT wording and
severity into a consistent convention. Patterns already visible that a
standard should resolve:

- **Inconsistent framing of outcome**: some messages bake in the recovery
  action (e.g. "...returning to FLM_IDLE" in `Flight_DockedProfile`) while
  structurally identical failures elsewhere don't (e.g. "MCB never confirmed
  motion" appears near-verbatim in `Flight_Profile`, `Flight_ManualMotion`,
  and `Flight_ReDock` with no consequence noted in any of them).
- **Severity drift for the same failure class**: unresponsive-hardware
  messages ("PU not responding...", "RPU not responding...") are
  consistently WARN today, but there's no codified rule tying failure class
  to severity — worth deciding explicitly rather than leaving it to
  precedent.
- **Capitalization/punctuation**: mostly consistent (no trailing periods),
  but with exceptions like "Starting LoRa failed!" (trailing `!`).
- **Dynamic (snprintf-built) messages bypass any template entirely** — e.g.
  `StartMCBMotion`'s "Deploying N revs" and `PURouter`'s RPU-reported error
  passthrough carry no subsystem/category prefix the way the static messages
  implicitly do via their originating state.

### Flight_CheckPU.cpp (PU status check)

| Message | Flag |
|---|---|
| PU not responding to status request | WARN |

### Flight_ManualMotion.cpp (TC-commanded manual reel motion)

| Message | Flag |
|---|---|
| Cannot perform motion, RA NAK | WARN |
| Never received RAAck | WARN |
| Motion commanded while motion ongoing | WARN |
| Motion start error | WARN |
| MCB never confirmed motion | WARN |
| Commanded motion stop | FINE |

### Flight_Profile.cpp (autonomous deploy → dock → dwell → retract profile)

| Message | Flag |
|---|---|
| Cannot perform motion, RA NAK | WARN |
| Never received RAAck | WARN |
| RPU not responding to go-measure command | WARN |
| No dock! Exceeded allowable number of redock attempts | CRIT |
| Motion commanded while motion ongoing | WARN |
| Motion start error | WARN |
| MCB never confirmed motion | WARN |
| Commanded motion stop in autonomous | WARN |
| Unable to schedule dwell | CRIT |
| MCB never powered off after profile | WARN |

### Flight_ReDock.cpp (redock sequence)

| Message | Flag |
|---|---|
| Motion commanded while motion ongoing | WARN |
| Motion start error | WARN |
| MCB never confirmed motion | WARN |
| Commanded motion stop | FINE |
| PU not responding to status request | WARN |

### Flight_PUOffload.cpp (RPU profile-record offload)

| Message | Flag |
|---|---|
| PU not successful in sending profile record | WARN |

### Flight_DockedProfile.cpp (docked profile with periodic offload)

| Message | Flag |
|---|---|
| Docked profile cancelled | FINE |
| Docked profile: invalid sample rate 0, returning to FLM_IDLE | WARN |
| Docked profile: invalid duration 0, returning to FLM_IDLE | WARN |
| Docked profile: period too long for sample rate, returning to FLM_IDLE | WARN |
| RPU not responding to go-measure command, returning to FLM_IDLE | WARN |
| Docked profile period complete; offloading and resuming measure *(or* `; offloading, finished` *at the end)* | FINE |
| RPU did not confirm standby; aborting docked profile, returning to FLM_IDLE | WARN |
| Docked profile aborted: RPU offload failed, returning to FLM_IDLE | WARN |
| Finished docked profile, returning to FLM_IDLE | FINE |

### Flight.cpp (general flight mode)

| Message | Flag |
|---|---|
| Entered flight error state | CRIT |

### Safety.cpp (safety mode)

| Message | Flag |
|---|---|
| Motion start error | WARN |

### PURouter.cpp (RPU message handling)

| Message | Flag |
|---|---|
| RPU NAKed go-measure command | WARN |
| RPU NAKed go-standby command | WARN |
| RPU acked reset | FINE |
| *(dynamic — RPU-reported error text via `RX_Error`)* | CRIT |

### StratoRachuts.cpp (setup / shared motion-start helper)

| Message | Flag |
|---|---|
| Error loading from EEPROM, reloaded default configuration | WARN |
| Starting LoRa failed! | WARN |
| *(dynamic — "Retracting N revs" / "Deploying N revs" / "Docking N revs" / "Reel in (no LW) N revs", from `StartMCBMotion`)* | FINE |

---

## Non-TM Zephyr messages (for contrast)

These use the Zephyr protocol but are **not** TMs (no `TM()`/`TM_String()`
call, no StateFlag/StateMess slots) — listed here only to avoid confusion
with the table above:

| Message | Sent by | Purpose |
|---|---|---|
| `IMR` | base `StratoCore` | Instrument mode request/announce on boot |
| `S` | base `StratoCore` | Safety-mode status ping |
| `RA` | base `StratoCore` | "Ready" acknowledgment |
| `IMAck` | base `StratoCore` | Ack of an incoming `IM` (instrument mode) message |
| `TCAck` | base `StratoCore` | Ack of an incoming telecommand (separate from the `RACHUTSTCACK` TM, which summarizes the *result*) |
