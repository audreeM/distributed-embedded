# Lab 5: wiring, build, and test plan

Code layout:

| Folder | What |
|---|---|
| `common/raft/` | Raft (election, log, state machine), the client and relay routing. Shared by all 3 nodes |
| `common/timesync/` | Time sync math (two-step REF, drift fit). Shared by all 3 nodes |
| `lab5_stm32/shared/` | Everything both STM32s run (CAN, dispatch, zone, raft thread, time, PPS/blinkers) |
| `lab5_stm32/drive/`, `lab5_stm32/steering/` | Just the motor / servo parts and the board pins |
| `lab5_pi/` | The cockpit program |

## 0. Laptop tests (no hardware)

```sh
cd common/raft/sim && make && ./raft_sim          # 9 scenarios, demo 1-3 + checkoffs 4, 5, 6-7, 9, 10
cd common/timesync/sim && make && ./ts_sim        # time sync: convergence, leader handover, relayed REF
```

Both should end with everything passing. `./raft_sim c5 -v` prints every raft step.

## 1. New wiring

Everything from lab 3 stays. Add these (LEDs through a resistor like the lab 3 ones; test points go to the scope):

| Signal | Pi (BCM GPIO / header pin) | N2 steering | N3 drive |
|---|---|---|---|
| LEADER LED | GPIO16 / pin 36 | PC10 (CN7-1) | PC10 (CN7-1) |
| SYNC LED | GPIO20 / pin 38 | PC12 (CN7-3) | PC12 (CN7-3) |
| REF_RX test point | GPIO21 / pin 40 | PC2 (CN7-35) | PC2 (CN7-35) |
| PPS test point | GPIO26 / pin 37 | PC3 (CN7-37) | PC3 (CN7-37) |
| HZ test point (hazard lamps) | GPIO19 / pin 35 | (FL lamp) | (RL lamp) |
| RELAY_TX test point | not wired | PB3 (lab 3) | PC4 (CN10, check silkscreen) |

Any STM32 pin that isn't wired is just skipped, so you can test before every wire is in.

## 2. Build and flash

**STM32s** (from `~/zephyrproject`, venv active):

```sh
west build -p always -b nucleo_f401re -d build_lab5_n2 distributed-embedded/lab5_stm32/steering
west flash -d build_lab5_n2          # with N2 plugged in
west build -p always -b nucleo_f401re -d build_lab5_n3 distributed-embedded/lab5_stm32/drive
west flash -d build_lab5_n3          # with N3 plugged in
```

Serial terminal at 115200. Keys: `s` raft status, `l` print the log, `e` report a test EVENT.

**Pi:**

```sh
# laptop: copy the shared code and the Pi code, keeping the layout
rsync -av --exclude 'build*' ~/zephyrproject/distributed-embedded/common \
      ~/zephyrproject/distributed-embedded/lab5_pi pi@<pi-ip>:~/de/
# Pi
cd ~/de/lab5_pi/bringup && ./can_up.sh     # after every reboot
cd ~/de/lab5_pi/src && make && sudo ./cockpit
```

Keys (then Enter): `s` status, `l` log, `e` test EVENT, `b` bad CTRL, `q` quit.
The Pi writes `raft_log.txt` (every committed entry) and `raft_state.txt` (term + vote) in `src/`.

**LEADER=random with a seed** (checkoff 8). All three nodes need the same settings:

```sh
west build -p always -b nucleo_f401re -d build_lab5_n2 distributed-embedded/lab5_stm32/steering -- -DRAFT_LEADER=0xFF -DRAFT_SEED=1
west build -p always -b nucleo_f401re -d build_lab5_n3 distributed-embedded/lab5_stm32/drive    -- -DRAFT_LEADER=0xFF -DRAFT_SEED=1
make clean && make RAFT_LEADER=0xFF RAFT_SEED=1      # on the Pi
```

From the sim (`./raft_sim seeds`): seed 1 -> N3 wins, seed 5 -> N2 wins, seed 2 -> the Pi wins.
Elections wait until each node has heard the other two (or 30 s), so boot order doesn't matter.
Go back to the default by building without the `-D` flags / with plain `make clean && make`.

## 3. Bring-up, one step at a time

Do these before the checkoff run. Each one only needs what the steps before it proved.

**3.1 One STM32 alone.** Flash it, open its terminal.
- Prints `raft: node 2, default leader 1, ...` and the RAM size of the raft node
- Every second: a status line, a `raft: follower term 1 leader N1 ...` line, a `sync: NOT synced` line, and a `PPS ...` line
- LEADER and SYNC LEDs off (it's waiting for the Pi, that's the default-leader rule)
- Hazards blinking at 4 Hz (the cockpit is missing)

**3.2 Pi alone, STM32s off.** `sudo ./cockpit`
- Prints `raft: cold start (no leader heard)` then `became leader`, LEADER LED on, SYNC LED on
- `candump -c any` shows `031` (REF) every 20 ms and `051` (empty APPEND) every 100 ms

**3.3 All three, proxy stopped (checkoff 1).** Start the STM32s, then the Pi.
- Pi: LEADER on. STM32s: LEADER off, SYNC on within a second
- STM32 status: `raft: follower term 1 leader N1`, `sync: synced, drift 1+x.xxxppm`
- All three in ERROR, hazards in step: scope FL (N2), RL (N3) and HZ (Pi)
- Start the proxy: everything goes NORMAL like lab 3

**3.4 Time sync.** Scope the three PPS pins.
- Rising edges line up. Expect tens of us between nodes, it should stay well under 1 ms
- Each node prints `PPS global ... | local ... | drift 1+x.xxxppm | last correction +n us`. Save a minute of these for the report (drift and jitter)
- `last correction` should stay small (tens of us) once it has settled

**3.5 Raft log.** On any node press `l`: `#1 LEADER_N1`. Pull one jumper and press `l` again on all three: same new entry at the same index everywhere.

## 4. Checkoff run (handout order)

| Step | Do | Look for |
|---|---|---|
| 1. Cold start | Power up with the proxy stopped, then start it | 3.3 above |
| 2. Bus traffic | `candump -c any` | REF `031` every 20 ms, empty APPEND `051` every 100 ms, no `041-043` (REQ_VOTE). REF bytes: `[seq] [term] [cycle] [5 bytes time, little endian us]` |
| 3. Blinkers | Left, right, cancel | FL and RL within 1 ms over ten edges (scope) |
| 4. Link faults | Pull one node's A jumper, put it back, then B | DEG LEDs like lab 3. Every console: `term 1` still. Pi prints `LOG #2 (term 1) CANLINK_2_A_DOWN` etc, one entry per change. `l` shows the same index on every node |
| 5. The hop | Pull N1 off B and N3 off A | Wheel still works. Term still 1. Two entries: `CANLINK_1_B_DOWN`, `CANLINK_3_A_DOWN`. N2's `relayed` count goes up. Press `e` on N2: `EVENT_N2_0x0F` shows up in N3's log (`l`) |
| 6. Pi down | Power off the Pi | STM32s ERROR + braked within 100 ms (lab 3). Within 1 s one STM32 has LEADER on and `candump` shows REF from it (`032` or `033`). Hazards at 4 Hz on FL and RL, in step |
| 7. Pi back | Power on the Pi, `sudo ./cockpit` | Pi prints `rejoining a running cluster`, then the events it missed, in order, each with `[replayed]`, then live ones. LEADER stays on the STM32. Hazards back to 2 Hz |
| 8. Random leader | Rebuild all three with `RAFT_LEADER=0xFF RAFT_SEED=1`, reboot | An STM32 leads. Power it off: new leader within 1 s, REF moves, no failure mode. Power off the Pi: failure mode, then replay on return. Rebuild with `RAFT_SEED=2` (Pi wins), repeat step 6 |
| 9. Relayed sync | STM32 leading (build all three with `RAFT_LEADER=2`, or power cycle the Pi so an STM32 takes over), pull N2 off A and N3 off B | The Pi relays (`relayed` in its status line). SYNC stays on at N3, REF_RX still toggles there, FL and RL within 1 ms. Put the jumpers back: no step on the edges |
| 10. Isolation + test button | Pull both jumpers on one node (lab 3), restore. Then test button | Term changes only if that node was leader, and only once (`s` on each node) |

**Demo scenarios (part 7)** are the same as `s1`, `s2`, `s3` in the sim: link failure with `LEADER=none` (build with `-DRAFT_LEADER=0`), link failure + recovery with `LEADER=cockpit`, and leader failure. Record `s` (getNetwork) from every node after each one.

## 5. If something's off

| Symptom | Check |
|---|---|
| STM32s never get a leader | Default leader mode waits for the Pi. Is `cockpit` running, and does its status say `leader`? |
| Pi says `rejoining` but there's no cluster running | Delete `raft_state.txt` and `raft_log.txt`, restart. (It decides by listening for REF for 300 ms) |
| `sync: NOT synced` on an STM32 | Is REF_RX toggling? If not, no REF reaches it (wiring, or relay not set up yet) |
| `last correction` jumps by hundreds of us | Note when. A few at a leader change or relay start are expected; steady jumps mean bad timestamps |
| A board keeps resetting | Watchdog. The terminal shows which thread stopped (status lines stop). Tell me what printed last |
| RAM | Both builds use ~89 KB of 96 KB with a 30000 entry log. To free some: `west build ... -- -DRAFT_LOG_SIZE=20000` (and `make RAFT_LOG_SIZE=20000` on the Pi, all nodes the same) |

## 6. Not done

- The extra credit (Redis + GUI).
- Measurements for the report: PPS spread from the scope, the PPS console lines for drift/jitter, and the timings in the checkoff table.
