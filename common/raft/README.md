# Raft (lab 5)

Shared Raft code for all three nodes. Nothing in here is Zephyr or Linux
specific, each node passes in its own functions through `struct raft_port`.

- `raft_config.h`: the config file from the handout (N, default_leader, heartbeat_ms, ...)
- `raft_msg.h`: CAN IDs, frame layouts, log entry format
- `raft_sm.c/h`: the network graph (addLink, dropLink, addNode, dropNode)
- `raft.c/h`: election, log replication, commit, redirect, getNetwork()
- `raft_client.c/h`: the client (reports link/node changes) and relay routing
- `sim/`: runs all 3 nodes on a laptop to test without hardware

Time sync is next door in `../timesync/`.

## Testing on a laptop

```sh
cd common/raft/sim
make && ./raft_sim          # runs every scenario
./raft_sim c5 -v            # one scenario with debug prints
./raft_sim seeds            # who wins the election for each seed (LEADER=random)
make asan && ./raft_sim_asan   # same thing with memory checking
```

Scenarios: `s1` `s2` `s3` (demos 1-3), `c4`, `c5`, `c67`, `c9`, `c10`, `c10L` (checkoffs).
The sim uses the real client and routing code; only the buses and a small
heartbeat monitor are fake.

## Using it on a node

```c
static struct raft_node raft;   // ~60 KB because of the log
static struct raft_client client;
static const struct raft_port port = {
	.now_us = my_now_us,
	.send = my_send_both_buses,  // REF: fill in the time per bus first (ts_ref_fill)
	.entry_committed = my_print, // optional
	.save_state = my_save,       // optional (Pi only)
	.log = my_log_line,          // optional
};

raft_init(&raft, MY_NODE_ID, &port, NULL, NULL);
// every 1-5 ms:                raft_tick(&raft);
// every raft frame received:   raft_receive_frame(&raft, frame.id, frame.data);
// every ~10 ms:                raft_client_update(&client, &raft, &obs, now);  // obs from HB_MON
// LEADER=random:               raft_peers_ready(&raft) once every node has been heard
// relay decision:              route_relay_bus(&eff, me, sender, bus) with eff from route_effective_net()
```

Call all of these from one thread (or under one mutex). See
`lab5_stm32/shared/src/raft_task.c` and `lab5_pi/src/raftnode.c` for the real thing.

## Limitations

- STM32s keep term/vote in RAM only, so after a reboot one could vote twice in
  the same term. Real Raft saves this to disk. The Pi does save it.
- The log isn't saved anywhere. A rebooted node gets it from the leader, which
  is why the Pi shows `[replayed]` entries in checkoff 7.
- When the log is full new changes are dropped (no snapshots).
- Terms are 1 byte (wraps after 255 elections).
