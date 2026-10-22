# Lab 3 cockpit zone (N1, Raspberry Pi)

The Pi is the cockpit zone. It takes the wheel stream from the laptop proxy over
UDP and turns it into CAN messages for the two STM32 zones, on both CAN A and
CAN B. It is also the blinker master, watches the other zones' heartbeats, and
sends the steering zone's force value back to the proxy.

In Lab 2 the Pi talked to one STM32 over a serial link. In Lab 3 that link is
replaced by the dual CAN bus, and some logic that used to run on the STM32
(the blinker state machine and the self-test button) moves to the Pi.

## Folder layout

| Path | What it is |
|---|---|
| `bringup/` | Scripts to set up the CAN interfaces. `setup_can.sh` once, `can_up.sh` after every reboot |
| `src/` | The cockpit program |
| `../common/can_catalog.h` | The CAN message format, shared with both STM32s |

## Build and run

```bash
# from the laptop: copy the shared header and the Pi code, keeping the layout
rsync -av --exclude 'build*' ~/zephyrproject/distributed-embedded/common \
      ~/zephyrproject/distributed-embedded/lab3_pi pi@<pi-ip>:~/de/

# on the Pi
cd ~/de/lab3_pi/bringup && ./can_up.sh   # after every reboot
cd ~/de/lab3_pi/src && make
sudo ./cockpit                           # sudo for realtime priority and GPIO
```

`can_up.sh` makes sure `can0` is always bus A (SPI0 CE0) and `can1` is bus B
(SPI0 CE1). The program prints a status line five times a second. Type `b` and
Enter to send one CTRL message with an out-of-range throttle (the "bad input"
checkoff step), or `q` to quit.

## What stayed the same from Lab 2

| Lab 2 | Lab 3 | Notes |
|---|---|---|
| `gpio_tp.c`, `gpio_tp.h` | same files | Added `tp_set()` so the LEDs can be set on or off, not just toggled |
| `state.h` (the wheel packet struct) | same file | Unchanged |
| `main.c`: `map_axis()`, `button()`, `scale()` | `wheel.c` | Same scaling and deadbands, now filling in the CAN command |
| `main.c`: UDP socket, `poll()` loop, 1 ms timer, Ctrl-C handling, realtime priority | `main.c` | Same structure, with the two CAN sockets added to the loop |
| Resend the command every 20 ms; stop when the wheel stream goes stale | `main.c` | Same rule, now for CTRL |
| `config.h`: wheel calibration, button numbers, deadbands | `config.h` | Same values |

## What changed

| Lab 2 | Lab 3 |
|---|---|
| Serial link to one STM32 (`serial.c`) | Two CAN buses to two STM32s (`canbus.c`) |
| Framed packets with CRC-16 and a byte parser (`common/link_proto.h`) | Fixed 8-byte CAN messages (`common/can_catalog.h`). CAN does the framing; CTRL carries a CRC-8 |
| One command message with everything in it | Separate messages: CTRL, BLINK, heartbeat, FAULT, FORCE |
| The STM32 sent a status frame back; the Pi printed it | Every zone sends a heartbeat; the Pi tracks each one per bus |
| The STM32 ran the blinker state machine and its own blink timer | The Pi runs the blinker state machine and sends a BLINK message on every lamp edge, so the front and rear lamps on different boards stay in sync |
| The STM32 handled the self-test button (sent as a bit in the command) | The Pi handles its own test button; each STM32 has its own hardware button |
| No force feedback | The steering zone's FORCE value is sent back to the proxy on UDP port 8001 every 50 ms |
| Test points UDP_RX, CMD_TX | Also CAN_TX_A, CAN_TX_B, the DEG_A / DEG_B status LEDs and an ERR LED |

Removed: `serial.c`, `link_proto.h` and its parser, and the status-frame printing.

## New code that is done

| File | What it does |
|---|---|
| `common/can_catalog.h` | Message IDs and byte layouts, packing and unpacking, CRC-8, range checks, and the duplicate filter that drops the second copy of each message |
| `canbus.c` | Opens `can0` and `can1`. Sends each message on bus A, then bus B, without ever blocking, so a disconnected bus can't hold up the other one |
| `main.c` | CTRL on every wheel update, heartbeat every 20 ms, FAULT, BLINK sending, receiving with duplicate dropping, force to the proxy, the status LEDs and test points |
| `util.h` | Time helpers (Pi clock in microseconds) |

The program already runs as it is: it sends CTRL and heartbeats, so the STM32
code can be tested against it. Until the work below is done it stays in the
error state (ERR LED on) and sends no BLINK messages.

## Still to do

Each of these has a `TODO(cockpit)` comment in the code that spells out the
exact rule.

1. **`health.c`: `health_update()`**. Turn the "last heard" times into:
   - which zones are alive (heard on at least one bus recently)
   - which zones have failed (3 heartbeats missed on both buses)
   - which bus is degraded (nothing from anyone on it for 60 ms while the
     other bus still works); this drives the DEG LEDs
   - whether the Pi is cut off from both buses entirely
2. **`zone.c`: `zone_update()` and `zone_on_fault()`**. The test button is
   already ported from Lab 2 (`on_test_button()`, from
   `lab2_stm32/src/sys_state.c`) and sets `z->self_test`. Still needed: the
   state machine itself. Start in ERROR; go to NORMAL once both STM32
   heartbeats and the wheel stream are there; back to ERROR when a zone fails
   or reports a fault; LOCAL_FAIL while `self_test` is set or there has been
   no wheel data for 100 ms. A failed cockpit stops sending everything,
   heartbeat included.
3. **`blinker.c`: the blink clock**. The Lab 2 state machine is already ported
   (`determine_next_state()`, `wheel_range()` and the event building, from
   `lab2_stm32/src/blinker.c`), so `b->mode` is correct. Still needed: every
   500 ms (250 ms for hazards) flip the lamps and return true so `main.c`
   sends a BLINK message, and send right away when the mode changes.
4. **Agree with the STM32 side** on one case: if the cockpit is the zone that
   failed, it goes quiet and nobody sends BLINK, so the STM32s have to flash
   their hazards on their own timers.
5. **Measure for the report** (Part 3): scope UDP_RX against CMD_TX for at
   least a minute, with and without `stress` running, and note the maximum and
   99th percentile.

## Testing without the STM32s

The program also sees CAN messages that other programs on the Pi send, so you
can fake the other zones from a second terminal:

```bash
# fake the steering zone's heartbeat on bus A, every 20 ms
s=0; while true; do cansend can0 101#0$(printf %X $s)01000000000000; s=$(((s+1)%16)); sleep 0.02; done

# fake a FAULT from the drivetrain zone (zone 3, reason 1 = test button)
cansend can0 020#0003010000000000

# fake a force value of +25 from the steering zone
cansend can0 300#0019000000000000
```

Once `health.c` is done, stopping the fake heartbeat should mark the steering
zone as failed after 3 periods, and sending it on `can0` only should mark bus B
as degraded.

## Test points and LEDs

| Row | Signal | GPIO (header pin) |
|---|---|---|
| 1, 2 | GND | (6, 9) |
| 3 | UDP_RX: toggles on each wheel packet | 23 (16) |
| 4 | CMD_TX: toggles once CTRL is queued on both buses | 17 (11) |
| 5 | CAN_TX_A: toggles on every send on bus A | 27 (13) |
| 6 | CAN_TX_B: toggles on every send on bus B | 22 (15) |
| 7 | DEG_A LED: on while bus A is degraded | 5 (29) |
| 8 | DEG_B LED: on while bus B is degraded | 6 (31) |
| 9 | ERR LED: on while the cockpit is not in NORMAL | 13 (33) |
