# Lab 5 cockpit zone (N1, Raspberry Pi)

Lab 3's cockpit program plus raft, relaying and time sync. Build, wiring and
the full test plan are in [../LAB5_TESTING.md](../LAB5_TESTING.md).

```bash
cd bringup && ./can_up.sh              # after every reboot
cd ../src && make && sudo ./cockpit    # sudo: realtime priority + GPIO
```

Keys (then Enter): `s` raft status, `l` print the log, `e` report a test EVENT,
`b` bad CTRL (lab 3), `q` quit.

## What changed from lab 3

| File | Change |
|---|---|
| `main.c` | Raft frames go to `raftnode.c`, REF also to time sync. Any node's frame gets relayed to the other bus when the routing table says the Pi is the only bridge. LEADER / SYNC LEDs, REF_RX test point, PPS lines. At startup it listens 300 ms for a running cluster (see below) |
| `raftnode.c` (new) | The raft node and its port: sends on both buses (REF gets its time per bus), saves term/vote to `raft_state.txt`, prints and logs every committed entry to `raft_log.txt` |
| `gtime.c` (new) | Global time: `common/timesync` with a mutex. `global_gettime()` |
| `edges.c` (new) | Realtime thread for PPS (whole seconds of global time) and HZ (hazard lamp) pins |
| `canbus.c` | Gets the kernel timestamp of every frame, and our own frames back when they finish sending (that's when a REF really went out). `canbus_send_on()` for one bus |
| `health.c` | Keeps each node's heartbeat health byte, builds the `raft_obs` the client and routing use |
| `zone.c` | No wheel data is now ERROR, not LOCAL_FAIL: the Pi keeps its heartbeat, hazards and raft going with the proxy stopped (checkoff 1). LOCAL_FAIL (silent) is only the test button |
| `blinker.c` | Blink edges are on the global time grid; BLINK carries the pattern's start time (~20 ms ahead) so every node switches together |
| `config.h` | Lab 5 pins and timing at the bottom |

## Cold start vs rejoining

When `cockpit` starts it listens for REF frames for 300 ms:
- **Nothing heard:** everyone is starting up. Old `raft_state.txt` / `raft_log.txt` are cleared and the Pi starts fresh, which makes it leader in term 1 with `LEADER=cockpit`.
- **REF heard:** a cluster is running and the Pi is coming back (checkoff 7). It loads its term and vote, comes back as a follower, and doesn't re-display entries it already showed before it went down. The entries it missed print with `[replayed]`.

## Pins

| Row | Signal | GPIO (header pin) |
|---|---|---|
| lab 3 | UDP_RX, CMD_TX, CAN_TX_A, CAN_TX_B, DEG_A, DEG_B, ERR | 23, 17, 27, 22, 5, 6, 13 |
| lab 5 | LEADER LED | 16 (36) |
| lab 5 | SYNC LED | 20 (38) |
| lab 5 | REF_RX | 21 (40) |
| lab 5 | PPS | 26 (37) |
| lab 5 | HZ (hazard lamps) | 19 (35) |
