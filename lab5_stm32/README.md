# Lab 5 STM32 zones (N2 steering, N3 drivetrain)

Build, flash, wiring and the test plan are in [../LAB5_TESTING.md](../LAB5_TESTING.md).

In lab 3 the two apps were near copies of each other. Now everything they
share lives in `shared/` and is built once per app with `NODE_ID` set by the
app's `CMakeLists.txt`:

| Folder | What |
|---|---|
| `shared/src/` | CAN, dispatch, zone + HB_MON + test button, heartbeat, relay, raft thread, global time, PPS + blinkers, main |
| `drive/` | `NODE_ID 3`: motor + PID (unchanged from lab 3), its pins |
| `steering/` | `NODE_ID 2`: servo + FORCE_TX (unchanged from lab 3), its pins |

Serial console at 115200: `s` raft status, `l` the log, `e` report a test EVENT.

## Threads

| Thread | Prio | Does |
|---|---|---|
| edges | 0 | PPS and blinker lamps from global time. Sleeps until 300 us before an edge, then spins on the 1 us clock |
| dispatch | 1 | Every received frame: HB_MON, dedup, raft frames to the raft thread, REF to time sync, relay decision, CTRL / BLINK / FAULT |
| motor / relay / steer | 2-4 | Lab 3 |
| test button, HB_MON, HB_TX, PID, FORCE_TX | 5-8 | Lab 3 |
| raft | 9 | Owns the raft node: frames, `raft_tick` every 2 ms, client + routing table every 10 ms, LEADER LED |
| console | 13 | Serial keys |
| main | 12 | Watchdog, status and PPS prints (printk is slow, so it's low priority) |

## What changed from lab 3

- **Relaying** uses the routing table from the raft log (part 5): copy a frame to the other bus when the sender isn't on it and some node is only on it. Lab 3's rule was "haven't heard the sender on the other bus for 60 ms".
- **Blinkers** run off global time: the cockpit's BLINK says which pattern and when it starts, and each board works out the edges itself. With the cockpit gone (failure mode) both STM32s run 4 Hz hazards on the same global grid.
- **Receive filters** take every ID, since any node may have to relay anything.
- **Own FAULTs** also go in the raft log as `EVENT_Nx_<reason>`.

## Pins added (both boards)

LEADER LED PC10 (CN7-1), SYNC LED PC12 (CN7-3), REF_RX PC2 (CN7-35), PPS PC3 (CN7-37).
N3 also gets RELAY_TX on PC4 (N2 already had it on PB3). Unwired pins are skipped.

## Memory

~89 KB of 96 KB RAM, ~60 KB of it the raft log (30000 entries x 2 bytes, the handout's `log_size`).
`west build ... -- -DRAFT_LOG_SIZE=20000` frees 20 KB if needed.
