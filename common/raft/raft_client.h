/*
 * Raft client (part 1): watches the network and reports changes to the
 * leader, plus the routing decision (part 5) built from the same information.
 * Portable C like the rest of common/raft.
 *
 * Each node builds a raft_obs from its heartbeat monitor (HB_MON) and passes
 * it in. Everything here uses only that struct, so the laptop sim can test
 * the exact same code.
 *
 * Where the information comes from:
 *  - node X alive: we got one of X's heartbeats recently, on either bus
 *    (relayed ones count too)
 *  - node X's links: X's own heartbeat says which of its buses are degraded
 *    (byte 2 of HB_Nx, from lab 3). X knows its own links better than anyone
 *    else, and that heartbeat can reach us through a relay even when we
 *    don't share a bus with X.
 */
#ifndef RAFT_CLIENT_H
#define RAFT_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "raft.h"

/* how long without any heartbeat from a node before we report NODE_X_DOWN.
 * Longer than lab 3's 60 ms so a relay has time to start after a second
 * link fails (part 5.2), otherwise we'd report a node down that's only cut off. */
#define RAFT_NODE_DOWN_MS 150

/* how often the client compares what it sees with getNetwork() and reports differences */
#define RAFT_CLIENT_PERIOD_MS 50

/* health bits, same as CAN_HEALTH_* in can_catalog.h: bit = that bus is degraded */
#define RAFT_HEALTH_BIT(bus) (1u << (bus))

struct raft_obs {
	bool heard_anyone;             /* we hear at least one other node (else we're cut off and can't report) */
	bool alive[RAFT_N + 1];        /* heartbeat from that node within RAFT_NODE_DOWN_MS (own slot ignored) */
	bool health_fresh[RAFT_N + 1]; /* we have a recent heartbeat payload from it (own slot: always true) */
	uint8_t health[RAFT_N + 1];    /* its degraded bits, own slot = our own */
};

struct raft_client {
	uint64_t next_us;
	uint64_t connected_since; /* when heard_anyone last went true, 0 = not connected */
};

/*
 * Call often (every tick is fine, it rate limits itself to every
 * RAFT_CLIENT_PERIOD_MS). Compares obs with the committed network and calls
 * raft_report_change() for each difference. Reports that are already in the
 * leader's log get dropped by the leader, so repeating them is harmless.
 */
void raft_client_update(struct raft_client *c, struct raft_node *n, const struct raft_obs *o,
			uint64_t now_us);

/*
 * The network we route with: the committed one from getNetwork(), with each
 * node's links replaced by what its latest heartbeat says. This covers the
 * gap between a link failing and the log entry for it committing (part 5.2).
 * Without it the relay couldn't start until the change committed, but the
 * change can't reach the leader without the relay.
 */
void route_effective_net(const struct net_state *committed, const struct raft_obs *o, uint8_t me,
			 struct net_state *eff);

/*
 * Should we copy a frame from `sender` that arrived on `from_bus` onto the
 * other bus? Yes if we're on both buses, the sender isn't on the other bus,
 * and some other node is only on the other bus. Returns the bus to send it
 * on, or -1.
 *
 * With 3 nodes only one node can ever pass this test for a given frame (the
 * one that isn't the sender or the cut off node), so frames never bounce
 * back and forth between two relays.
 */
int route_relay_bus(const struct net_state *eff, uint8_t me, uint8_t sender, int from_bus);

#endif /* RAFT_CLIENT_H */
