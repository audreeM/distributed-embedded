/*
 * Raft state machine (part 4): the network graph.
 *
 * 3 nodes, each connected to bus A and bus B (figure 3). Everything starts up
 * and the log only records changes. addNode() brings a node back with no links,
 * so a node coming back after NODE_X_DOWN also needs CANLINK_X_A/B_UP entries.
 *
 * sm_apply() runs each committed entry through the 4 functions below.
 * getNetwork() in raft.h reads the result.
 */
#ifndef RAFT_SM_H
#define RAFT_SM_H

#include <stdbool.h>
#include <stdint.h>

#include "raft_config.h"

struct net_state {
	bool node_up[RAFT_N + 1];    /* [node 1..N], index 0 unused */
	bool link_up[RAFT_N + 1][2]; /* [node][RAFT_BUS_A / RAFT_BUS_B] */
	bool cockpit_ok;             /* false = cockpit error, failure mode */
};

void sm_init(struct net_state *s);

/* the 4 functions from part 4. return true if something changed */
bool addLink(struct net_state *s, uint8_t node, uint8_t bus);
bool dropLink(struct net_state *s, uint8_t node, uint8_t bus);
bool addNode(struct net_state *s, uint8_t node);
bool dropNode(struct net_state *s, uint8_t node);

/* apply one log entry, true if something changed */
bool sm_apply(struct net_state *s, uint8_t entry);

/* which buses both a and b are on. bit 0 = A, bit 1 = B */
uint8_t net_shared_buses(const struct net_state *s, uint8_t a, uint8_t b);

/* prints like "N1 A- B+ | N2 A+ B+ | N3 down | cockpit OK" */
void net_str(const struct net_state *s, char *buf, int len);

#endif /* RAFT_SM_H */
