# Pi ↔ STM32 link (Lab 2 stand-in for CAN)

Source of truth: [common/link_proto.h](common/link_proto.h). Both sides include it, so framing, CRC and the parser are the same code on the Pi and the STM32. Unit tests: `make -C lab2_pi test`.

## Physical

| Signal | Raspberry Pi | STM32 Nucleo-F401RE |
|---|---|---|
| Pi → STM32 (commands) | pin 8, GPIO14 TXD | PA12, USART6_RX (CN10-12) |
| STM32 → Pi (status) | pin 10, GPIO15 RXD | PA11, USART6_TX (CN10-14) |
| Ground | pin 6 | GND |

(pins might change!! upddate if needed)

UART 115200 8N1, 3.3 V on both ends. USART2 (PA2/PA3) is **not** used, because the ST-Link VCP drives those pins and carries the printk console.

## Frame

```
A5 5A | TYPE | LEN | SEQ | PAYLOAD[LEN] | CRC lo | CRC hi
```
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD. Multi-byte fields are little-endian.
- SEQ is an 8-bit counter per sender and per message type (as in design doc §7.5) that increments on every frame. The STM32 echoes the last accepted command SEQ in STATUS.
- **Every payload is exactly 8 bytes (LEN = 8)**, the CAN 2.0A maximum, so each frame type maps 1:1 onto a future CAN message (TYPE ≈ CAN ID, LEN ≈ DLC). Unused bytes are reserved and must be 0. Each frame is 15 bytes, 1.3 ms at 115200.

| TYPE | Name | Direction | When |
|---|---|---|---|
| 0x10 | CMD | Pi → STM32 | every UDP update, ≤ 20 ms refresh |
| 0x20 | STATUS | STM32 → Pi | every 20 ms (heartbeat) |
| 0x21 | CURRENTS | STM32 → Pi | every 20 ms, right after STATUS |

### CMD, TYPE 0x10, Pi → STM32
| Byte | Field | Range |
|---|---|---|
| 0 | throttle % | 0–100 |
| 1 | brake % | 0–100 (any value > 0 means brake) |
| 2 | steer, int8 | −100 full left … +100 full right |
| 3 | buttons | bit0 left, bit1 right, bit2 self-test; bits 3–7 must be 0; left and right must not both be set |
| 4–7 | reserved | must be 0 |

The Pi does all scaling from raw wheel counts, using the calibration in `lab2_pi/src/config.h`. The STM32 only ever sees engineering units, the same encoding as the Lab 1 CAN catalog (BRAKE_CMD/DRIVE_CMD/STEER_CMD/BLINK_CMD).

### STATUS, TYPE 0x20, STM32 → Pi, every 20 ms (heartbeat)
| Byte | Field |
|---|---|
| 0 | state: 0 ERROR, 1 NORMAL |
| 1 | faults: bit0 POWERUP, bit1 LINK_LOST, bit2 BAD_CMD, bit3 SELF_TEST |
| 2 | SEQ of the last accepted command |
| 3 | malformed frames dropped (CRC/LEN/gap), saturates at 255 |
| 4 | out-of-range commands rejected, saturates at 255 |
| 5–7 | reserved (0) |

### CURRENTS, TYPE 0x21, STM32 → Pi, every 20 ms, sent right after STATUS
| Bytes | Field |
|---|---|
| 0–1 | left motor current, mA (uint16 LE) |
| 2–3 | right motor current, mA |
| 4–5 | servo current, mA |
| 6–7 | reserved (0) |

STATUS + CURRENTS together are the handout's "status frame every 20 ms with current-sensor readings and the zone's state". They are split because the combined payload (11 bytes) exceeds the 8-byte CAN 2.0A limit. On CAN, STATUS becomes the design doc's HEARTBEAT and CURRENTS a new message.

**Design doc impact:** §8.1 assumes DLC = 4 for all frames. At DLC = 8 a worst-case stuffed frame is 135 bits instead of 95 (44 overhead + 64 data + 24 stuff + 3 IFS), so §9's 750 frames/s becomes ≈ 101 kbit/s, **≈ 20 % per bus** instead of 14.25 %. That's still under the 50 % limit (R4.6). Record it in the revision history.

## Synchronisation and bad input

- **Resync.** The receiver hunts for `A5 5A`. TYPE must be one of the three above and LEN must be 8, or the frame is dropped at once.
- **Truncated frame.** If more than 2 ms (STM32) or 10 ms (Pi) pass between two bytes of one frame, the partial frame is thrown away. Frames are ≥ several ms apart, so a frame cut short by an unplug cannot swallow the start of the next one. Anything that does get mangled fails the CRC.
- **Malformed** (bad CRC, bad LEN, gap): dropped and counted, *not acted on*. Unplugging and replugging always produces a few of these, and a real loss of link is already caught by the watchdog.
- **Out of range** (CRC valid, a field outside the table above, or a reserved byte ≠ 0): rejected, never applied, does not feed the watchdog, and sets **BAD_CMD** → ERROR. Deviation from design doc §10.6: malformed frames are dropped instead of forcing the error state.
- **Overrun.** The STM32 UART ISR queues valid frames in a 4-deep `k_msgq`. If it is full, the *oldest* frame is dropped. Stale throttle is worthless and the newest command wins. The ISR never blocks.

## Timing

| What | Value | Requirement |
|---|---|---|
| Pi command | on every UDP packet, plus a resend if 20 ms pass without one | every update, ≤ 50 ms |
| Pi stops sending | wheel stream silent for 100 ms | cold start / proxy loss → STM32 fails safe |
| STM32 link timeout | 70 ms after the last accepted command (3 missed 20 ms refreshes + margin) | ≤ 150 ms (handout), ≤ 100 ms (checkoff) |
| STM32 STATUS + CURRENTS | 20 ms periodic k_timer, back to back (2.6 ms of wire time) | 20 ms ± 10 % |

## Fault state (STM32 `sys_state`)

ERROR while any fault bit is set; it boots with POWERUP set.
- POWERUP, LINK_LOST and BAD_CMD clear after **2 consecutive** accepted commands.
- SELF_TEST (wheel button → `buttons` bit2): rising edge with a 50 ms leading-edge lockout.
  - A single press while not in self-test sets SELF_TEST on the next command (µs after CMD_RX).
  - While in self-test, **two presses within 500 ms** clear it. The press that caused the failure does not count.
- In ERROR, actuator code must disable motor PWM, apply dynamic braking and run 2 Hz hazards.

## Tasks (link portion of the task table)

| Task | Kind | Period / trigger | Priority | Deadline | Talks to |
|---|---|---|---|---|---|
| UART RX/TX ISR | ISR | per byte | IRQ | < 1 byte time (87 µs) | parser → `rx_q`; TX ring buffer |
| link_rx | thread | on each frame | 1 | < 2 ms (brake/throttle path) | `rx_q` → latest cmd, watchdog, sys_state, wakes `link_wait()` |
| link watchdog | k_timer (one-shot) | 70 ms after last good cmd | ISR | 70 ms | sys_state (LINK_LOST) |
| status TX | k_timer (periodic) | 20 ms | ISR | 20 ms ± 2 ms | STATUS + CURRENTS → TX ring buffer → UART |
| main (placeholder) | thread | 10 ms / on wake | 10 | — | console printk, LD2 |

## Test points

| Point | Where | Toggles on |
|---|---|---|
| UDP_RX | Pi GPIO23 (pin 16) | each valid 276-byte wheel packet |
| CMD_TX | Pi GPIO24 (pin 18) | just before each command `write()` |
| CMD_RX | STM32 PC8 (CN10-2) | each CRC-valid command frame, in the UART ISR |
