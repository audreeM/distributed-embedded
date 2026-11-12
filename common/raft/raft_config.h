/*
 * Raft config for lab 5 (the "config file" from part 2). All three nodes
 * include this same file so they always have the same settings.
 *
 * LEADER and SEED change between checkoff runs, so they're build flags:
 *
 *   Zephyr:  west build ... -- -DEXTRA_CFLAGS="-DRAFT_DEFAULT_LEADER=0xFF -DRAFT_SEED=7"
 *   Pi:      make CFLAGS+="-DRAFT_DEFAULT_LEADER=0xFF -DRAFT_SEED=7"
 *
 * handout name           macro
 * N                      RAFT_N
 * default_leader         RAFT_DEFAULT_LEADER (node ID, RAFT_LEADER_NONE or RAFT_LEADER_RANDOM)
 * heartbeat_ms           RAFT_HEARTBEAT_MS
 * missed_heartbeats      RAFT_MISSED_HEARTBEATS
 * election_timeout_jitter_ms  RAFT_ELECTION_JITTER_MS
 * log_size               RAFT_LOG_SIZE
 * nodes                  RAFT_NODE_* and raft_node_name()
 * Debug                  RAFT_DEBUG
 */
#ifndef RAFT_CONFIG_H
#define RAFT_CONFIG_H

/* nodes, same numbers as lab 3 (CAN_ZONE_N1..N3) */
#define RAFT_NODE_COCKPIT  1
#define RAFT_NODE_STEERING 2
#define RAFT_NODE_DRIVE    3

/* N = number of nodes, majority = N/2 + 1 */
#define RAFT_N        3
#define RAFT_MAJORITY (RAFT_N / 2 + 1)

/*
 * default_leader:
 *   1, 2 or 3           that node is leader in term 1, no election ("leader setup mode")
 *   RAFT_LEADER_NONE    election at boot, random timeouts seeded from the clock ("default mode")
 *   RAFT_LEADER_RANDOM  election at boot, random timeouts seeded from RAFT_SEED, so the
 *                       same seed picks the same winner if all nodes boot together (checkoff 8)
 */
#define RAFT_LEADER_NONE   0x00
#define RAFT_LEADER_RANDOM 0xFF
#ifndef RAFT_DEFAULT_LEADER
#define RAFT_DEFAULT_LEADER RAFT_NODE_COCKPIT
#endif
#ifndef RAFT_SEED
#define RAFT_SEED 1
#endif

/* heartbeat_ms: how often the leader sends REF (also the time sync period) */
#define RAFT_HEARTBEAT_MS 20

/* missed_heartbeats: follower times out after this many heartbeats + jitter */
#define RAFT_MISSED_HEARTBEATS 3

/* election_timeout_jitter_ms: random 0..this added to each timeout */
#define RAFT_ELECTION_JITTER_MS 39

/* log_size: max entries in the log, 2 bytes each */
#ifndef RAFT_LOG_SIZE
#define RAFT_LOG_SIZE 30000
#endif

/* Debug: print elections, appends and commits */
#ifndef RAFT_DEBUG
#define RAFT_DEBUG 1
#endif

/* our own timing settings (not in the handout) */
#define RAFT_APPEND_IDLE_MS     100 /* empty APPEND this often when nothing new */
#define RAFT_APPEND_RETRY_MS    2   /* when catching a node up, one entry every 2 ms */
#define RAFT_FOLLOWER_ACTIVE_MS 200 /* stop sending entries to a follower that hasn't acked in this long */
#define RAFT_QUORUM_LOST_MS     300 /* leader steps down after this long without a majority */

static inline const char *raft_node_name(unsigned id)
{
	switch (id) {
	case RAFT_NODE_COCKPIT:  return "cockpit";
	case RAFT_NODE_STEERING: return "steering";
	case RAFT_NODE_DRIVE:    return "drive";
	default:                 return "?";
	}
}

#endif /* RAFT_CONFIG_H */
