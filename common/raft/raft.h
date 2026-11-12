/*
 * Raft for lab 5. The same code runs on the Pi and both STM32s, no Zephyr or
 * Linux calls in here. Each node fills in a raft_port (clock, CAN send,
 * callbacks) and then calls:
 *
 *   raft_init()           at boot
 *   raft_tick()           every 1-5 ms
 *   raft_receive_frame()  for every Raft frame received on either bus
 *   raft_report_change()  when the client sees a link or node change
 *
 * Not thread safe, so call all of these from one thread or under one mutex.
 *
 * Differences from normal Raft:
 *  - default leader: every node boots in term 1 with the configured leader
 *    so there's no election at startup. Followers don't start their election
 *    timer until they've heard from a leader once (the Pi boots slowly).
 *  - pre-vote: when a follower times out it first asks "would you vote for
 *    me?" without changing its term. Anyone who still hears the leader says
 *    no. That way a node that just lost a link can't bump the term.
 *  - check-quorum: if a leader stops getting acks from a majority for 300 ms
 *    it steps down (so the LEADER LED goes off on an isolated leader).
 *  - only term and vote get saved, and only on the Pi. The log isn't saved;
 *    a rebooted node gets it back from the leader.
 */
#ifndef RAFT_H
#define RAFT_H

#include <stdbool.h>
#include <stdint.h>

#include "../can_catalog.h" /* can_seq_filter */
#include "raft_config.h"
#include "raft_msg.h"
#include "raft_sm.h"

struct raft_port {
	void *ctx; /* passed back to every function below */

	/* required */
	uint64_t (*now_us)(void *ctx);                           /* local clock */
	void (*send)(void *ctx, uint16_t id, const uint8_t d[8]); /* send on both buses */

	/* optional, can be NULL */
	void (*entry_committed)(void *ctx, uint16_t index, uint8_t term, uint8_t entry, bool replayed);
	void (*fill_ref)(void *ctx, uint8_t d[8]); /* time sync fills in REF bytes 4-7 */
	void (*save_state)(void *ctx, uint8_t term, uint8_t voted_for);
	void (*log)(void *ctx, const char *line); /* debug prints, no newline at the end */
};

struct raft_opts {
	uint8_t default_leader; /* node ID, RAFT_LEADER_NONE or RAFT_LEADER_RANDOM */
	uint32_t seed;          /* used when default_leader == RAFT_LEADER_RANDOM */
};

/* term and vote loaded from a file at boot, NULL if there isn't one */
struct raft_saved {
	uint8_t term;
	uint8_t voted_for;
};

enum raft_role {
	RAFT_FOLLOWER,
	RAFT_PRECANDIDATE,
	RAFT_CANDIDATE,
	RAFT_LEADER,
};

struct raft_log_entry {
	uint8_t term;
	uint8_t entry;
};

struct raft_node {
	uint8_t id;
	const struct raft_port *port;
	struct raft_opts opts;

	/* election */
	enum raft_role role;
	uint8_t term;
	uint8_t voted_for; /* 0 = nobody */
	uint8_t leader;    /* 0 = don't know */
	bool heard_leader; /* no elections until this is true (default leader mode) */
	uint8_t votes;     /* bit i set = node i voted for us */
	uint64_t last_leader_us;
	uint64_t deadline_us; /* election timeout */
	uint32_t rng;

	/* only used while leader */
	uint64_t next_ref_us, next_append_us, next_idle_us;
	uint16_t ref_cycle;
	uint16_t next_index[RAFT_N + 1];  /* next entry to send each follower */
	uint16_t match_index[RAFT_N + 1]; /* last entry each follower has */
	uint64_t last_ack_us[RAFT_N + 1];
	bool quorum_seen;

	/* drop duplicate frames (bus A + bus B + relayed copy), per msg type and sender */
	struct can_seq_filter rx_filter[RAFT_MSG_REDIRECT + 1][RAFT_N + 1];

	/* log */
	uint16_t last_index;
	uint16_t commit_index;
	uint16_t applied_index;
	uint16_t replay_until;     /* entries up to here were already committed when we got them */
	struct net_state net;      /* committed network, what getNetwork() returns */
	struct net_state pending_net; /* network including uncommitted entries, leader uses it to skip duplicates */
	struct raft_log_entry log[RAFT_LOG_SIZE + 1]; /* starts at 1, log[0] is a dummy with term 0 */

	uint8_t tx_seq[8];
};

/* raft_leader_add_entry errors */
#define RAFT_ERR_NOT_LEADER (-1)
#define RAFT_ERR_LOG_FULL   (-2)
#define RAFT_ERR_INVALID    (-3)

/*
 * Call once at boot. my_id is 1/2/3. opts = NULL uses raft_config.h.
 * saved = term/vote loaded from a file, or NULL.
 */
void raft_init(struct raft_node *n, uint8_t my_id, const struct raft_port *port,
	       const struct raft_opts *opts, const struct raft_saved *saved);

/*
 * Call every 1-5 ms. The leader sends heartbeats and log entries from here,
 * followers check if the leader went quiet, and committed entries get applied.
 */
void raft_tick(struct raft_node *n);

/* Call for every CAN frame received on either bus. Non-raft frames are ignored. */
void raft_receive_frame(struct raft_node *n, uint16_t id, const uint8_t d[8]);

/*
 * Leader only. Adds an entry to the log and returns its index. Returns 0 if
 * the entry wouldn't change anything (already reported), or RAFT_ERR_*.
 */
int raft_leader_add_entry(struct raft_node *n, uint8_t entry);

/* For the client: adds the entry if we're leader, otherwise sends an EVENT to the leader. */
void raft_report_change(struct raft_node *n, uint8_t entry);

/* committed network graph (part 4) */
void getNetwork(const struct raft_node *n, struct net_state *out);

static inline bool raft_is_leader(const struct raft_node *n) { return n->role == RAFT_LEADER; }
static inline uint8_t raft_leader_id(const struct raft_node *n) { return n->leader; }
static inline uint8_t raft_term(const struct raft_node *n) { return n->term; }
static inline uint16_t raft_last_index(const struct raft_node *n) { return n->last_index; }
static inline uint16_t raft_commit_index(const struct raft_node *n) { return n->commit_index; }
const char *raft_role_str(enum raft_role r);

/* copies out log[index], false if index is out of range */
bool raft_log_get(const struct raft_node *n, uint16_t index, struct raft_log_entry *out);

#endif /* RAFT_H */
