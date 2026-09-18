# CAN protocol — motor bus

Classic CAN 2.0A (11-bit IDs) @ 500 kbit/s. All multi-byte values little-endian.
Motor ids 0–3. Nodes: Raspberry Pi (MCP2515 HAT) + 4× B-G431B-ESC1 (`firmware/`).

| ID | dir | payload |
|---|---|---|
| `0x000` | Pi → all | **ESTOP**: dlc 0. Every motor: target=0 + disable. |
| `0x080` | Pi → all | **Setpoints**: 4× int16, 0.01 rad/s; motor *i* reads bytes 2i..2i+1. |
| `0x100+id` | Pi → motor | **Command**: byte0 = opcode, args below. |
| `0x200+id` | motor → Pi | **Telemetry**: see below. Default every 100 ms. |
| `0x280+id` | motor → Pi | **Diag**: control-loop internals, off by default (OP_DIAG). |
| `0x300+id` | motor → Pi | **Hello**: at boot and on PING. |
| `0x380+id` | motor → Pi | **Gain value**: reply to OP_GAIN (get or set). |

## Command opcodes (`0x100+id`)

| op | name | args |
|---|---|---|
| `0x00` | STOP | — (target=0, stays enabled → brakes to zero) |
| `0x01` | VEL | f32 rad/s at bytes 1–4 |
| `0x02` | ENABLE | — (also clears watchdog-tripped) |
| `0x03` | DISABLE | — (target=0 + freewheel) |
| `0x04` | CURLIM | f32 A — Iq command cap (`PID_velocity.limit`) |
| `0x05` | TELEM | u16 ms telemetry period, 0 = off |
| `0x06` | TERM | u8 0/1 — onboard 120 Ω termination |
| `0x07` | PING | — → motor replies with hello frame |
| `0x08` | WATCHDOG | u16 ms; disable if no cmd/setpoint received for that long. 0 = off |
| `0x09` | GAIN | u8 param idx (+ f32 to set); always replies current value on `0x380+id` |
| `0x0A` | DIAG | u16 ms diag frame period, 0 = off |

### GAIN param indices

| idx | param | idx | param |
|---|---|---|---|
| 0 | vel PID P | 6 | current-q PID P |
| 1 | vel PID I | 7 | current-q PID I |
| 2 | vel PID D | 8 | current-q LPF Tf |
| 3 | vel PID output_ramp | 9 | current-d PID P |
| 4 | vel LPF Tf | 10 | current-d PID I |
| 5 | vel PID limit (Iq cap, A) | 11 | current-d LPF Tf |

Volatile — reverts to firmware defaults on reboot; hardcode tuned values.

## Telemetry (`0x200+id`, dlc 8)

| bytes | field | scale |
|---|---|---|
| 0–1 | int16 velocity | 0.01 rad/s (clamps at ±327.67) |
| 2–3 | int16 target | 0.01 rad/s |
| 4–5 | int16 Iq | 1 mA |
| 6 | u8 flags | bit0 enabled, bit1 initFOC ok, bit2 watchdog tripped |
| 7 | int8 Uq | 0.1 V |

## Diag (`0x280+id`, dlc 8)

| bytes | field | scale |
|---|---|---|
| 0–1 | int16 Iq setpoint (vel-PID output) | 1 mA |
| 2–3 | int16 Id measured | 1 mA |
| 4–5 | int16 Uq | 10 mV |
| 6–7 | u16 electrical angle | 2π/65536 rad |

Vel-PID P-term is reconstructable offline from telemetry (`P·(target−vel)`);
I-term = Iq setpoint − P-term. A frozen electrical angle while Iq sits at the
cap means the sensor isn't seeing movement; a rotating one means slip/open phase.

## Gain value (`0x380+id`, dlc 5)

byte0 = param idx, bytes 1–4 = f32 current value. Sent for every OP_GAIN
(a set is acked with the post-write value).

## Hello (`0x300+id`, dlc 8)

| bytes | field |
|---|---|
| 0–3 | u32 MCU UID word0 |
| 4 | u8 firmware version |
| 5 | u8 init flags: bit0 currentSense ok, bit1 initFOC ok |
| 6 | u8 motor id |
| 7 | reserved |

An unexpected hello = that board rebooted (brownout, watchdog reset, reflash).

## Notes

- ESC hardware filters accept only `0x000`, `0x080`, `0x100+id` — motors never
  see each other's telemetry.
- ID space: lower = higher CAN priority (ESTOP wins arbitration, then
  setpoints, then commands, then telemetry).
- ESC firmware: FDCAN1 in classic mode, 170 MHz kernel clock, prescaler 20,
  seg1 14 / seg2 2 (sample point 88%). MCP2515 side must stay classic CAN —
  never send FD frames on this bus.
