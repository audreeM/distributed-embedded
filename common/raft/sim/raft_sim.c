/*
 * Tests raft.c on a laptop. Fakes 3 nodes on 2 CAN buses and runs through
 * the checkoff scenarios (cutting wires, powering nodes off).
 *
 *   make && ./raft_sim            run every scenario
 *   ./raft_sim c5 -v              one scenario with debug prints
 *   ./raft_sim seeds              who wins the election for each seed
 *
 * relay_sim() and client_sim() are simple fake versions of what part C builds:
 *  - relay_sim: a node on both buses copies a raft frame to the other bus if
 *    the sender isn't on that bus and someone else is only on that bus.
 *  - client_sim: every 50 ms each node checks what it sees (with a 60-66 ms
 *    delay like HB_MON) against getNetwork() and reports anything different.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raft.h"

#define N      RAFT_N
#define STEP   100  /* us of virtual time per loop */
#define TICK   1000 /* raft_tick period, us */
#define BUS_US 300  /* frame latency on the wire + receive path */
#define QMAX   8192

static uint64_t now;
static int verbose;

static struct raft_node nodes[N + 1];
static struct raft_port ports[N + 1];
static struct raft_saved disk[N + 1];
static const bool persists[N + 1] = {false, true, false, false}; /* only the Pi has a disk */
static struct raft_opts opts;

/* what's actually plugged in / powered on */
static bool alive[N + 1];
static bool wire[N + 1][2];

/* what HB_MON would see (delayed) */
struct obs {
	bool cur, seen;
	uint64_t since;
};
static struct obs obs_link[N + 1][2], obs_node[N + 1];

struct frame {
	uint64_t at;
	uint8_t from, bus;
	uint16_t id;
	uint8_t d[8];
};
static struct frame q[QMAX];
static int qn;
static unsigned long sent_by_msg[8];

/* port functions */

static uint64_t port_now(void *ctx)
{
	(void)ctx;
	return now;
}

static void enqueue(int from, int bus, uint16_t id, const uint8_t d[8])
{
	if (!alive[from] || !wire[from][bus] || qn >= QMAX) {
		return;
	}
	q[qn] = (struct frame){.at = now + BUS_US, .from = (uint8_t)from, .bus = (uint8_t)bus, .id = id};
	memcpy(q[qn].d, d, 8);
	qn++;
}

static void port_send(void *ctx, uint16_t id, const uint8_t d[8])
{
	int me = (int)(intptr_t)ctx;
	uint8_t s;

	sent_by_msg[raft_msg_from_id(id, &s)]++;
	enqueue(me, RAFT_BUS_A, id, d);
	enqueue(me, RAFT_BUS_B, id, d);
}

static void port_log(void *ctx, const char *line)
{
	(void)ctx;
	if (verbose) {
		printf("%9.1f ms  %s\n", now / 1000.0, line);
	}
}

static void port_save(void *ctx, uint8_t term, uint8_t voted)
{
	int me = (int)(intptr_t)ctx;

	if (persists[me]) {
		disk[me] = (struct raft_saved){term, voted};
	}
}

static void port_commit(void *ctx, uint16_t idx, uint8_t term, uint8_t entry, bool replayed)
{
	int me = (int)(intptr_t)ctx;
	char s[24];

	if (verbose) {
		printf("%9.1f ms  N%d display: #%u (term %u) %s%s\n", now / 1000.0, me, idx, term,
		       raft_entry_str(entry, s, sizeof(s)), replayed ? "  [replayed]" : "");
	}
}

static void boot(int i)
{
	alive[i] = true;
	ports[i] = (struct raft_port){
		.ctx = (void *)(intptr_t)i,
		.now_us = port_now,
		.send = port_send,
		.entry_committed = port_commit,
		.save_state = port_save,
		.log = port_log,
	};
	raft_init(&nodes[i], (uint8_t)i, &ports[i], &opts, persists[i] ? &disk[i] : NULL);
}

/* fake relay and client */

static void relay_sim(int r, const struct frame *f)
{
	struct net_state net;
	uint8_t s;
	int other = !f->bus;

	if ((f->d[0] & RAFT_FLAG_RELAYED) || !wire[r][0] || !wire[r][1] ||
	    raft_msg_from_id(f->id, &s) == RAFT_MSG_NONE || s == r) {
		return;
	}
	getNetwork(&nodes[r], &net);
	if (net.link_up[s][other]) {
		return; /* sender is already on that bus */
	}
	for (int y = 1; y <= N; y++) {
		if (y != s && y != r && net.node_up[y] && net.link_up[y][other] &&
		    !net.link_up[y][f->bus]) {
			uint8_t d[8];

			memcpy(d, f->d, 8);
			d[0] |= RAFT_FLAG_RELAYED;
			enqueue(r, other, f->id, d);
			return;
		}
	}
}

static void observe(struct obs *o, bool truth, uint64_t debounce)
{
	if (truth != o->cur) {
		o->cur = truth;
		o->since = now;
	}
	if (now - o->since >= debounce) {
		o->seen = o->cur;
	}
}

static void update_observations(void)
{
	for (int x = 1; x <= N; x++) {
		for (int b = 0; b < 2; b++) {
			observe(&obs_link[x][b], alive[x] && wire[x][b], 60000);
		}
		observe(&obs_node[x], alive[x] && (wire[x][0] || wire[x][1]), 66000);
	}
}

static void client_sim(int j)
{
	struct net_state net;

	getNetwork(&nodes[j], &net);
	for (int x = 1; x <= N; x++) {
		if (obs_node[x].seen != net.node_up[x] && (x != j || obs_node[x].seen)) {
			raft_report_change(&nodes[j], raft_e_node((uint8_t)x, obs_node[x].seen));
			continue; /* links follow once the node entry commits */
		}
		if (!net.node_up[x]) {
			continue;
		}
		for (int b = 0; b < 2; b++) {
			if (obs_link[x][b].seen != net.link_up[x][b]) {
				raft_report_change(&nodes[j], raft_e_link((uint8_t)x, (uint8_t)b,
									obs_link[x][b].seen));
			}
		}
	}
	if (obs_node[RAFT_NODE_COCKPIT].seen != net.cockpit_ok) {
		raft_report_change(&nodes[j], obs_node[RAFT_NODE_COCKPIT].seen ? RAFT_E_COCKPIT_OK
									     : RAFT_E_COCKPIT_ERROR);
	}
}

/* main loop */

static void deliver(void)
{
	int keep = 0;

	for (int i = 0; i < qn; i++) {
		struct frame f = q[i];

		if (f.at > now) {
			q[keep++] = f;
			continue;
		}
		for (int j = 1; j <= N; j++) {
			if (j != f.from && alive[j] && wire[j][f.bus]) {
				raft_receive_frame(&nodes[j], f.id, f.d);
				relay_sim(j, &f);
			}
		}
	}
	/* anything sent during this loop got added to the end and kept, since it's not due yet */
	qn = keep;
}

static void run_until(uint64_t t_ms)
{
	while (now < t_ms * 1000) {
		now += STEP;
		update_observations();
		deliver();
		if (now % TICK == 0) {
			for (int i = 1; i <= N; i++) {
				if (alive[i]) {
					raft_tick(&nodes[i]);
				}
			}
		}
		if (now % 50000 == 0) {
			for (int i = 1; i <= N; i++) {
				if (alive[i]) {
					client_sim(i);
				}
			}
		}
	}
}

/* scenarios */

enum act { CUT, FIX, KILL, BOOT };

struct step {
	unsigned at_ms;
	enum act act;
	int node, bus;
};

struct scenario {
	const char *name, *what;
	uint8_t leader_mode;
	unsigned end_ms;
	struct step steps[8];
	int nsteps;
	int expect_term;   /* -1 = don't check */
	int expect_leader; /* 0 = any, -1 = any STM32 */
	bool expect_no_vote;
};

static const struct scenario scenarios[] = {
	{"s1", "Demo 1: link failure, default mode (no predefined leader)", RAFT_LEADER_RANDOM, 2000,
	 {{1000, CUT, 2, RAFT_BUS_A}}, 1, -1, 0, false},
	{"s2", "Demo 2: link failure and recovery, leader setup mode", RAFT_NODE_COCKPIT, 2500,
	 {{500, CUT, 3, RAFT_BUS_A}, {1500, FIX, 3, RAFT_BUS_A}}, 2, 1, 1, true},
	{"s3", "Demo 3: leader failure, leader setup mode", RAFT_NODE_COCKPIT, 2000,
	 {{500, KILL, 1, 0}}, 1, 2, -1, false},
	{"c4", "Checkoff 4: N2 off A, restore, then off B, restore (no election)", RAFT_NODE_COCKPIT, 3000,
	 {{500, CUT, 2, RAFT_BUS_A}, {1500, FIX, 2, RAFT_BUS_A}, {1600, CUT, 2, RAFT_BUS_B},
	  {2500, FIX, 2, RAFT_BUS_B}}, 4, 1, 1, true},
	{"c5", "Checkoff 5: N1 off B, N3 off A (relay through N2, term unchanged)", RAFT_NODE_COCKPIT,
	 2500, {{500, CUT, 1, RAFT_BUS_B}, {1000, CUT, 3, RAFT_BUS_A}}, 2, 1, 1, false},
	{"c67", "Checkoff 6-7: Pi down, STM32 leads, Pi back as follower", RAFT_NODE_COCKPIT, 3500,
	 {{500, KILL, 1, 0}, {1500, BOOT, 1, 0}}, 2, 2, -1, false},
	{"c10", "Checkoff 10: isolate follower N3, restore (term unchanged)", RAFT_NODE_COCKPIT, 2500,
	 {{500, CUT, 3, RAFT_BUS_A}, {500, CUT, 3, RAFT_BUS_B}, {1500, FIX, 3, RAFT_BUS_A},
	  {1500, FIX, 3, RAFT_BUS_B}}, 4, 1, 1, false},
	{"c10L", "Checkoff 10: isolate leader N1, restore (term changes once)", RAFT_NODE_COCKPIT, 2500,
	 {{500, CUT, 1, RAFT_BUS_A}, {500, CUT, 1, RAFT_BUS_B}, {1500, FIX, 1, RAFT_BUS_A},
	  {1500, FIX, 1, RAFT_BUS_B}}, 4, 2, -1, false},
};

static void reset(uint8_t leader_mode, uint32_t seed)
{
	now = 0;
	qn = 0;
	memset(sent_by_msg, 0, sizeof(sent_by_msg));
	memset(disk, 0, sizeof(disk));
	opts = (struct raft_opts){.default_leader = leader_mode, .seed = seed};
	for (int i = 1; i <= N; i++) {
		wire[i][0] = wire[i][1] = true;
		obs_link[i][0] = obs_link[i][1] = (struct obs){true, true, 0};
		obs_node[i] = (struct obs){true, true, 0};
	}
	for (int i = 1; i <= N; i++) {
		boot(i);
	}
}

static int leader_count(int *who)
{
	int c = 0;

	for (int i = 1; i <= N; i++) {
		if (alive[i] && raft_is_leader(&nodes[i]) && (wire[i][0] || wire[i][1])) {
			c++;
			*who = i;
		}
	}
	return c;
}

static bool logs_agree(void)
{
	for (int i = 1; i <= N; i++) {
		for (int j = i + 1; j <= N; j++) {
			struct raft_node *a = &nodes[i], *b = &nodes[j];
			uint16_t upto;

			if (!alive[i] || !alive[j]) {
				continue;
			}
			upto = a->commit_index < b->commit_index ? a->commit_index : b->commit_index;
			for (uint16_t k = 1; k <= upto; k++) {
				if (a->log[k].term != b->log[k].term || a->log[k].entry != b->log[k].entry) {
					printf("  MISMATCH N%d/N%d at #%u\n", i, j, k);
					return false;
				}
			}
		}
	}
	return true;
}

static void dump(void)
{
	char s[24], net[96];

	for (int i = 1; i <= N; i++) {
		struct net_state ns;

		if (!alive[i]) {
			printf("  N%d (%s): powered off\n", i, raft_node_name(i));
			continue;
		}
		getNetwork(&nodes[i], &ns);
		net_str(&ns, net, sizeof(net));
		printf("  N%d %-9s term %u leader N%u commit %u/%u  getNetwork: %s\n", i,
		       raft_role_str(nodes[i].role), raft_term(&nodes[i]), raft_leader_id(&nodes[i]),
		       raft_commit_index(&nodes[i]), raft_last_index(&nodes[i]), net);
	}
	for (int i = 1; i <= N; i++) {
		if (alive[i]) {
			printf("  log (N%d):", i);
			for (uint16_t k = 1; k <= nodes[i].commit_index; k++) {
				printf(" #%u:%s", k, raft_entry_str(nodes[i].log[k].entry, s, sizeof(s)));
			}
			printf("\n");
			break;
		}
	}
}

static bool run_scenario(const struct scenario *sc)
{
	int who = 0, leaders;
	bool ok = true;
	uint8_t term = 0;

	printf("\n=== %s: %s\n", sc->name, sc->what);
	reset(sc->leader_mode, 1);
	for (int k = 0; k < sc->nsteps; k++) {
		const struct step *st = &sc->steps[k];
		static const char *names[] = {"cut", "fix", "power off", "power on"};

		run_until(st->at_ms);
		if (verbose) {
			printf("%9.1f ms  ---- %s N%d%s\n", now / 1000.0, names[st->act], st->node,
			       st->act <= FIX ? (st->bus ? " bus B" : " bus A") : "");
		}
		switch (st->act) {
		case CUT:  wire[st->node][st->bus] = false; break;
		case FIX:  wire[st->node][st->bus] = true; break;
		case KILL: alive[st->node] = false; break;
		case BOOT: boot(st->node); break;
		}
	}
	run_until(sc->end_ms);
	dump();

	leaders = leader_count(&who);
	for (int i = 1; i <= N; i++) {
		if (alive[i] && nodes[i].term > term) {
			term = nodes[i].term;
		}
	}
	if (leaders != 1) {
		printf("  FAIL: %d leaders\n", leaders);
		ok = false;
	}
	if (!logs_agree()) {
		printf("  FAIL: logs disagree\n");
		ok = false;
	}
	if (sc->expect_term >= 0 && term != sc->expect_term) {
		printf("  FAIL: term %u, expected %d\n", term, sc->expect_term);
		ok = false;
	}
	if (sc->expect_leader > 0 && who != sc->expect_leader) {
		printf("  FAIL: leader N%d, expected N%d\n", who, sc->expect_leader);
		ok = false;
	}
	if (sc->expect_leader < 0 && who == RAFT_NODE_COCKPIT) {
		printf("  FAIL: leader is the cockpit, expected an STM32\n");
		ok = false;
	}
	if (sc->expect_no_vote && sent_by_msg[RAFT_MSG_REQ_VOTE] != 0) {
		printf("  FAIL: %lu REQ_VOTE frames\n", sent_by_msg[RAFT_MSG_REQ_VOTE]);
		ok = false;
	}
	printf("  frames: REF %lu, APPEND %lu, ACK %lu, REQ_VOTE %lu, VOTE %lu, EVENT %lu\n",
	       sent_by_msg[RAFT_MSG_REF], sent_by_msg[RAFT_MSG_APPEND],
	       sent_by_msg[RAFT_MSG_APPEND_ACK], sent_by_msg[RAFT_MSG_REQ_VOTE],
	       sent_by_msg[RAFT_MSG_VOTE], sent_by_msg[RAFT_MSG_EVENT]);
	printf("  %s\n", ok ? "PASS" : "FAIL");
	return ok;
}

static void seed_table(void)
{
	printf("LEADER=random: winner per seed (all nodes boot together)\n");
	for (uint32_t seed = 1; seed <= 20; seed++) {
		int who = 0;

		reset(RAFT_LEADER_RANDOM, seed);
		run_until(500);
		leader_count(&who);
		printf("  seed %2u -> N%d (%s), term %u\n", seed, who, raft_node_name(who),
		       raft_term(&nodes[who ? who : 1]));
	}
}

int main(int argc, char **argv)
{
	const char *only = NULL;
	int failed = 0, ran = 0;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0) {
			verbose = 1;
		} else {
			only = argv[i];
		}
	}
	if (only && strcmp(only, "seeds") == 0) {
		seed_table();
		return 0;
	}
	for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
		if (only && strcmp(only, scenarios[i].name) != 0) {
			continue;
		}
		ran++;
		failed += !run_scenario(&scenarios[i]);
	}
	if (ran == 0) {
		fprintf(stderr, "unknown scenario %s\n", only);
		return 2;
	}
	printf("\n%d/%d scenarios passed\n", ran - failed, ran);
	return failed ? 1 : 0;
}
