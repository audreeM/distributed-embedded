/* Raft client and routing. See raft_client.h. */
#include "raft_client.h"

static bool link_up_from_health(uint8_t health, int bus)
{
	return (health & RAFT_HEALTH_BIT(bus)) == 0;
}

void raft_client_update(struct raft_client *c, struct raft_node *n, const struct raft_obs *o,
			uint64_t now_us)
{
	struct net_state net;
	uint8_t me = n->id;

	if (now_us < c->next_us) {
		return;
	}
	c->next_us = now_us + (uint64_t)RAFT_CLIENT_PERIOD_MS * 1000u;

	/* cut off from everyone: nobody would get our reports anyway, and we
	 * can't tell a dead node from our own broken wires */
	if (!o->heard_anyone) {
		c->connected_since = 0;
		return;
	}
	if (c->connected_since == 0) {
		c->connected_since = now_us;
	}
	/* just booted or just reconnected: we haven't had time to hear every
	 * heartbeat yet, so "not heard" doesn't mean "down" yet */
	bool settled = now_us - c->connected_since >= (uint64_t)RAFT_NODE_DOWN_MS * 1000u;

	getNetwork(n, &net);

	for (uint8_t x = 1; x <= RAFT_N; x++) {
		bool alive = (x == me) ? true : o->alive[x];

		if (!alive && !settled) {
			continue;
		}
		/* node up/down first. a node that comes back has no links until
		 * its CANLINK_X_Y_UP entries go in, so do those on a later round */
		if (alive != net.node_up[x]) {
			raft_report_change(n, raft_e_node(x, alive));
			continue;
		}
		if (!alive || !o->health_fresh[x]) {
			continue;
		}
		for (uint8_t bus = 0; bus < 2; bus++) {
			bool up = link_up_from_health(o->health[x], bus);

			if (up != net.link_up[x][bus]) {
				raft_report_change(n, raft_e_link(x, bus, up));
			}
		}
	}

	/* failure mode: the cockpit's heartbeats stopped (or came back) */
	{
		bool ok = (me == RAFT_NODE_COCKPIT) ? true : o->alive[RAFT_NODE_COCKPIT];

		if (ok != net.cockpit_ok && (ok || settled)) {
			raft_report_change(n, ok ? RAFT_E_COCKPIT_OK : RAFT_E_COCKPIT_ERROR);
		}
	}
}

void route_effective_net(const struct net_state *committed, const struct raft_obs *o, uint8_t me,
			 struct net_state *eff)
{
	*eff = *committed;
	for (uint8_t x = 1; x <= RAFT_N; x++) {
		bool fresh = (x == me) || o->health_fresh[x];

		if ((x == me || o->alive[x]) && !eff->node_up[x]) {
			eff->node_up[x] = true; /* heard it, even if NODE_X_UP isn't committed yet */
		}
		if (!fresh || !eff->node_up[x]) {
			continue;
		}
		for (int bus = 0; bus < 2; bus++) {
			eff->link_up[x][bus] = link_up_from_health(o->health[x], bus);
		}
	}
}

int route_relay_bus(const struct net_state *eff, uint8_t me, uint8_t sender, int from_bus)
{
	int other = !from_bus;

	if (sender < 1 || sender > RAFT_N || sender == me || me < 1 || me > RAFT_N) {
		return -1;
	}
	/* we have to be on both buses to bridge them */
	if (!eff->link_up[me][from_bus] || !eff->link_up[me][other]) {
		return -1;
	}
	/* the sender reaches the other bus by itself */
	if (eff->node_up[sender] && eff->link_up[sender][other]) {
		return -1;
	}
	/* someone needs it: a live node that's only on the other bus */
	for (uint8_t y = 1; y <= RAFT_N; y++) {
		if (y != me && y != sender && eff->node_up[y] && eff->link_up[y][other] &&
		    !eff->link_up[y][from_bus]) {
			return other;
		}
	}
	return -1;
}
