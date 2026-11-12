# Raft (lab 5 part A)

Shared Raft code for all three nodes. Nothing in here is Zephyr or Linux
specific, each node passes in its own functions through `struct raft_port`.

- `raft_config.h`: the config file from the handout (N, default_leader, heartbeat_ms, ...)
- `raft_msg.h`: CAN IDs, frame layouts, log entry format
- `raft_sm.c/h`: the network graph (addLink, dropLink, addNode, dropNode)
- `raft.c/h`: election, log replication, commit, redirect, getNetwork()
- `sim/`: runs all 3 nodes on a laptop to test without hardware

## Testing on a laptop

```sh
cd common/raft/sim
make && ./raft_sim          # runs every scenario
./raft_sim c5 -v            # one scenario with debug prints
./raft_sim seeds            # who wins the election for each seed (LEADER=random)
make asan && ./raft_sim_asan   # same thing with memory checking
```

Scenarios: `s1` `s2` `s3` (demos 1-3), `c4`, `c5`, `c67`, `c10`, `c10L` (checkoffs).

## Using it on a node

```c
static struct raft_node raft;   // ~60 KB because of the log
static const struct raft_port port = {
	.now_us = my_now_us,         // STM32: k_ticks_to_us_floor64(k_uptime_ticks()), Pi: CLOCK_MONOTONIC
	.send = my_send_both_buses,
	.entry_committed = my_print_entry, // Pi prints entries here, add "[replayed]" if replayed is true
	.fill_ref = ts_fill_ref,     // from time sync
	.save_state = NULL,          // Pi saves term/vote to a file, STM32s leave NULL
	.log = my_log_line,          // printk / printf
};

raft_init(&raft, MY_NODE_ID, &port, NULL, NULL);
// every 1-5 ms:                raft_tick(&raft);
// every raft frame received:   raft_receive_frame(&raft, frame.id, frame.data);
// client sees a change:        raft_report_change(&raft, raft_e_link(node, bus, up));
```

Things to know:
- Call `raft_tick`, `raft_receive_frame` and `raft_report_change` from the same thread (or use a mutex).
- RX filters need to let through IDs 0x031-0x067 on both CAN controllers.
- REF frames go to both `raft_receive_frame()` and time sync.
- LEADER LED = `raft_is_leader(&raft)`.
- The relay has to forward raft frames too, with `RAFT_FLAG_RELAYED` set in byte 0.
  Duplicates get dropped by seq number so it's fine if a node gets a frame 3 times.
- Check STM32 RAM after building (`west build -t ram_report`). If it doesn't
  fit, build with a smaller log: `-DRAFT_LOG_SIZE=...`.

## Limitations

- STM32s keep term/vote in RAM only, so after a reboot one could vote twice in
  the same term. Real Raft saves this to disk. The Pi does save it.
- The log isn't saved anywhere. A rebooted node gets it from the leader, which
  is why the Pi shows `[replayed]` entries in checkoff 7.
- Terms are 1 byte (wraps after 255 elections).
- The sim's relay and client are simplified, they know exactly what's
  connected (after a delay) instead of using real heartbeats.
