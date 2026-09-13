# RS41 regeneration telecommand (TC 186, RPUREGENRS41)

Status: **implemented**. Design doc for kalnajslab-org/StratoCore_RACHUTS#21 — kept as a record of the
decisions behind it, since most of them aren't visible from the diff alone.

## Context

Issue #21 asked for a telecommand to trigger an RS41 regeneration cycle on the RPU. Per the issue
thread, no duration parameter is needed, and the final call was that the command should be **allowed
during a docked profile**, not just "docked but idle" — terminating and restarting a profile just to
regen would be worse operationally than letting it run mid-measurement.

Two sibling instruments already do the equivalent thing, and this mirrors their pattern:
- `StratoCore_LPC` (RS41 attached locally): TC `REGENRS41` sets a `Set_rs41regen` flag, consumed in
  `rs41Action()` right before the normal per-tick `decoded_sensor_data()` poll, where it calls
  `_rs41.recondition()`.
- `StratoCore_RATS` → `ECU`: TC `RATSRS41REGEN` sends a JSON command (`rs41Regen: true`) over the
  RATS↔ECU link; `ECU_Lib.cpp` calls `rs41.recondition()` on receipt.

The RPU is architecturally like the ECU here — the RS41 is wired to the RPU, and RACHuTS commands it
over the dock serial link via the binary `RPUComm`/`SerialComm` protocol — so this adds a new
`RPU_REGEN_RS41` message to that protocol and wires it up the same way `RPU_RESET` already is, plus
the LPC-style deferred-flag pattern on the RPU firmware side.

## Design

### No synchronous "is it docked" check

The dock serial link only reaches the RPU when physically docked, but there's no cheap, reliable way to
confirm that inside a single `TCHandler()` call. RACHuTS does track a docked flag —
`pibConfigs.pu_docked` (an `EEPROMData<bool>`, so it survives a reboot) — but that's only ever the
*last-known* state, written by `PUDock()`/`PUUndock()` as a side effect of other traffic. The one place
that needs a *real* answer, `Flight_Profile.cpp`'s `ST_VERIFY_DOCK`, doesn't just read the flag: it
first runs the multi-tick `Flight_CheckPU()` state machine (send `RPU_SEND_STATUS`, wait, retry once)
and only trusts `pu_docked` once a fresh response has actually landed. That round-trip can't be inlined
into one synchronous TC handler call.

So `RPUREGENRS41` follows the same pattern already used by every other RPU TC (`RPURESET`,
`RPUGOSTANDBY`, `RPUGOMEASURE`): no docked check, just send it. If undocked, no ACK/NAK ever arrives —
ground sees silence rather than a false-confidence success ack, which is itself the signal (same
behavior/limitation those existing TCs already have).

### The gate that matters: RS41 must actually be powered

On the RPU side, `dockComms()` NAKs the request unless `rpu_state == RPUState::MEASURE &&
sensorsEnabled.rs41`. This isn't a policy check, it's a hardware one: `enterStandby()` calls
`powerdownSensors()`, which drives `RS41_ENABLE` low, so in STANDBY the RS41 board has no power at all.
Calling `recondition()` against an unpowered board would just block the RPU's main loop for the RS41
serial read's ~1s timeout and return nothing. `sensorsEnabled.rs41` matters independently of `MEASURE`
too — ground can choose which sensors are powered per `RPU_GO_MEASURE`/`PROFILE`, so even a measuring
RPU may have the RS41 disabled for that session.

Requiring `MEASURE` happens to match the issue's own conclusion ("allowed during a docked profile, not
idle standby") but the reasoning is about the pin, not about profiles.

**Considered and deferred:** powering the RS41 on just long enough to run the regen, then back off,
so ground could regen without a full `MEASURE` session. Not implemented — there's no cheap "power on
for a moment" path today. The only thing that brings the RS41 up from unpowered is `RS41::init()`
(`platformio/RS41/src/RS41.cpp`), a one-shot boot routine that blocks 2+ seconds (power-cycle the enable
pin, wait for a boot banner, a further 1s settle delay the code says is required "without which the
RS41 does not seem to respond", then retried meta-data reads) — a materially bigger stall than
`recondition()`'s own ~1s. Every existing caller of `recondition()` (LPC, RATS/ECU, and
`recondition()`'s own implementation, which re-primes the RSD polling pipeline on exit) also assumes an
RS41 that's already actively sampling. Doing this properly would need new `RS41` library work (a real
lightweight wake, not reusing `init()`) plus an async multi-tick state machine on the RPU side, since a
multi-second power-up can't block the main loop the way the current one-line pending-flag check does.
Expect this to resurface as its own request.

### RPU-side: deferred flag, not an inline call

`recondition()` blocks on a serial read and re-primes the RS41's `RSD` polling pipeline on exit, so it
has to run in place of the normal per-tick `decoded_sensor_data()` poll, not inline from message
dispatch (which would stall dock-comms handling for up to a second). `dockComms()` sets a pending flag;
`tickMeasure()` consumes it immediately before the tick's `decoded_sensor_data()` call — the same
call ordering `StratoCore_LPC`'s `rs41Action()` uses:

```cpp
// RPU.cpp, file-scope
static bool rs41_regen_pending = false;

// dockComms(), ASCII-message switch
case RPU_REGEN_RS41: {
  bool accepted = (rpu_state == RPUState::MEASURE) && sensorsEnabled.rs41;
  if (accepted) {
    rs41_regen_pending = true;
  }
  rpucomm.TX_Ack(RPU_REGEN_RS41, accepted);
  return false;
}

// tickMeasure(), right before the existing poll
if (rs41_regen_pending) {
  Serial.println(rs41.recondition().c_str());
  rs41_regen_pending = false;
}
RS41::RS41SensorData_t sensor_data = rs41.decoded_sensor_data(false);
```

### Telemetry generated, end to end

1. **`RACHUTSTCACK`** (immediate, synchronous, from the existing TC-ack path in `TCHandler.cpp`): FINE
   "TC RPU RS41 Regen" — confirms only that the command was *sent* over the dock link, not that the RPU
   received or accepted it.
2. **Async text TM from `HandlePUAck()`**, only if the RPU's actual ACK/NAK arrives: FINE "RPU RS41
   regen started" on ACK, WARN "RPU NAKed RS41 regen (...)" on NAK. Unlike `RPU_GO_MEASURE`/
   `RPU_GO_STANDBY` (routine mode changes, where only the NAK path gets a ground TM and success is a
   local-only `log_nominal`), this is a one-off, deliberately ground-commanded maintenance action, so
   both outcomes are worth confirming — same reasoning as `RPU_RESET`'s unconditional ack TM.
3. **No dedicated "regen complete" TM.** `recondition()` is fire-and-forget; neither `StratoCore_LPC`
   nor the `ECU` firmware send one either. Per the issue thread itself ("the status flags and the data
   will indicate that the recondition is in progress"), progress/completion is inferred from the
   ordinary RPU status report and RS41 record stream already flowing.

### Enum placement: appended, not inserted

`RPUMessages_t` (`RPUComm.h`) has no explicit numeric values, so inserting `RPU_REGEN_RS41` in the
middle (e.g. next to `RPU_RESET`, which is where it reads best) would silently renumber every message
after it. It's appended at the end of the enum instead — same effect, zero renumbering risk.
`Telecommand_t` (`StrateoleXML/Telecommand.h`) does use explicit values, so `RPUREGENRS41 = 186` was
free to go in its natural spot, right after `RPUGOMEASURE = 185`.

## Files changed

**Shared libraries** — per standing project practice, each edited identically in its canonical repo
*and* in every project's vendored `.pio/libdeps` copy (commits are made from the canonical repos):
- `kalnajslab-org/RPUcomm` (`RPUcomm.h`) — appended `RPU_REGEN_RS41` to `RPUMessages_t`. Vendored into
  both `StratoCore_RACHUTS` (`rachuts` env) and `RPU` (`RPU` and `RPUtest` envs).
- `kalnajslab-org/StrateoleXML` (`Telecommand.h`) — added `RPUREGENRS41 = 186` to `Telecommand_t`.
  Vendored into `StratoCore_RACHUTS` only (the RPU firmware doesn't parse Zephyr telecommands).

**StratoCore_RACHUTS**
- [src/TCHandler.cpp](../src/TCHandler.cpp) — `case RPUREGENRS41` next to `RPURESET`, fire-and-forget
  `TX_ASCII(RPU_REGEN_RS41)`.
- [src/PURouter.cpp](../src/PURouter.cpp) — `HandlePUAck()` case reporting ACK/NAK to ground (see
  Telemetry above).
- [docs/TelecommandCribSheet.md](TelecommandCribSheet.md) — TC 186 row and notes.

**RPU firmware** (`kalnajslab-org/RPU`)
- `src/RPU/RPU.cpp` — `rs41_regen_pending` file-scope flag, the `RPU_REGEN_RS41` case in `dockComms()`,
  and the check in `tickMeasure()` (see Design above).

## Out of scope

- Powering up the RS41 solely to service a regen request outside `MEASURE` — see "Considered and
  deferred" above.
- Reconciling pre-existing drift between the canonical `StrateoleXML` repo and `StratoCore_RACHUTS`'s
  vendored copy (RATS/LPC-specific commands, comment-formatting differences) — this change only added
  the one new enum value to both, unrelated drift was left alone.

## Verification

- `pio run -e rachuts` (StratoCore_RACHUTS) and `pio run -e RPU` / `-e RPUtest` (RPU) all build clean.
  **Done.**
- Bench, undocked: send TC 186 → FINE `RACHUTSTCACK` (send attempted), but no follow-up ACK/NAK TM ever
  arrives (matches existing `RPURESET`/`RPUGOSTANDBY` behavior when undocked).
- Bench, docked, RPU in STANDBY: send TC 186 → RPU NAKs, WARN "RPU NAKed RS41 regen" TM.
- Bench, docked, RPU in MEASURE (`RPUGOMEASURE`/TC 185, or a docked profile) with RS41 enabled: send TC
  186 → RPU ACKs, FINE "RPU RS41 regen started" TM, and the RPU's own serial log shows the
  `rs41.recondition()` response string.
- Confirm normal RS41 sample records keep flowing for a few ticks after a regen (no stuck `RSD`
  pipeline).
