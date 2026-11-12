#include <stdio.h>

#include "raft_msg.h"
#include "raft_sm.h"

static bool valid(uint8_t node) { return node >= 1 && node <= RAFT_N; }

void sm_init(struct net_state *s)
{
	*s = (struct net_state){0};
	for (uint8_t n = 1; n <= RAFT_N; n++) {
		s->node_up[n] = true;
		s->link_up[n][RAFT_BUS_A] = true;
		s->link_up[n][RAFT_BUS_B] = true;
	}
	s->cockpit_ok = true;
}

/* link back up */
bool addLink(struct net_state *s, uint8_t node, uint8_t bus)
{
	if (!valid(node) || bus > 1 || s->link_up[node][bus]) {
		return false;
	}
	s->link_up[node][bus] = true;
	return true;
}

/* link down, the node stays */
bool dropLink(struct net_state *s, uint8_t node, uint8_t bus)
{
	if (!valid(node) || bus > 1 || !s->link_up[node][bus]) {
		return false;
	}
	s->link_up[node][bus] = false;
	return true;
}

/* node back, with no links until addLink */
bool addNode(struct net_state *s, uint8_t node)
{
	if (!valid(node) || s->node_up[node]) {
		return false;
	}
	s->node_up[node] = true;
	s->link_up[node][RAFT_BUS_A] = false;
	s->link_up[node][RAFT_BUS_B] = false;
	return true;
}

/* node gone, and all its links too */
bool dropNode(struct net_state *s, uint8_t node)
{
	if (!valid(node) || !s->node_up[node]) {
		return false;
	}
	s->node_up[node] = false;
	s->link_up[node][RAFT_BUS_A] = false;
	s->link_up[node][RAFT_BUS_B] = false;
	return true;
}

bool sm_apply(struct net_state *s, uint8_t entry)
{
	struct raft_edec d = raft_entry_decode(entry);

	switch (d.kind) {
	case RAFT_EK_NODE:
		return d.up ? addNode(s, d.node) : dropNode(s, d.node);
	case RAFT_EK_LINK:
		return d.up ? addLink(s, d.node, d.bus) : dropLink(s, d.node, d.bus);
	case RAFT_EK_COCKPIT:
		if (s->cockpit_ok == d.up) {
			return false;
		}
		s->cockpit_ok = d.up;
		return true;
	default:
		return false; /* NONE, LEADER and EVENT don't change the graph */
	}
}

uint8_t net_shared_buses(const struct net_state *s, uint8_t a, uint8_t b)
{
	uint8_t mask = 0;

	if (!valid(a) || !valid(b) || !s->node_up[a] || !s->node_up[b]) {
		return 0;
	}
	for (uint8_t bus = 0; bus < 2; bus++) {
		if (s->link_up[a][bus] && s->link_up[b][bus]) {
			mask |= (uint8_t)(1 << bus);
		}
	}
	return mask;
}

void net_str(const struct net_state *s, char *buf, int len)
{
	int k = 0;

	for (uint8_t n = 1; n <= RAFT_N && k < len; n++) {
		if (s->node_up[n]) {
			k += snprintf(buf + k, len - k, "N%u A%c B%c | ", n,
				      s->link_up[n][RAFT_BUS_A] ? '+' : '-',
				      s->link_up[n][RAFT_BUS_B] ? '+' : '-');
		} else {
			k += snprintf(buf + k, len - k, "N%u down | ", n);
		}
	}
	if (k < len) {
		snprintf(buf + k, len - k, "cockpit %s", s->cockpit_ok ? "OK" : "ERROR");
	}
}
