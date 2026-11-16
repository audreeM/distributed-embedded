/*
 * Tests the Raft code on a laptop. Fakes 3 nodes on 2 CAN buses and runs
 * through the checkoff scenarios (cutting wires, powering nodes off).
 *
 *   make && ./raft_sim            run every scenario
 *   ./raft_sim c5 -v              one scenario with debug prints
 *   ./raft_sim seeds              who wins the election for each seed
 *
 * Uses the real raft.c, raft_sm.c and raft_client.c (client + relay routing).
 * The only fake parts are the buses and a mini heartbeat monitor: every node
 * sends a heartbeat every 20 ms carrying its bus health (like lab 3's HB_Nx),
 * and works out "bus degraded" / "node alive" from what it receives, the same
 * way the boards' HB_MON does.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raft.h"
#include "raft_client.h"

#define N      RAFT_N
#define STEP   100  /* us of virtual time per loop */
#define TICK   1000 /* raft_tick period, us */
#define BUS_US 300  /* frame latency on the wire + receive path */
#define QMAX   8192

#define HB_ID(x)        (0x100 + (x) - 1) /* CAN_ID_HB_N1.. from can_catalog.h */
#define HB_PERIOD_US    20000
#define DEGRADED_US     50000 /* bus silent this long while the other works = degraded */
#define HEALTH_FRESH_US 60000

static uint64_t now;
static int verbose;

static struct raft_node nodes[N + 1];
static struct raft_port ports[N + 1];
static struct raft_client clients[N + 1];
static struct raft_saved disk[N + 1];
static const bool persists[N + 1] = {false, true, false, false}; /* only the Pi has a disk */
static struct raft_opts opts;

/* what's actually plugged in / powered on */
static bool alive[N + 1];
static bool wire[N + 1][2];

/* each node's heartbeat monitor */
struct hbmon {
	uint64_t boot;
	uint64_t last_rx[2];        /* any frame from another node, per bus */
	uint64_t last_hb[N + 1];    /* heartbeat from node x, any bus (relayed too) */
	uint8_t health[N + 1];      /* last health byte from node x */
	uint8_t hb_seq;
	uint64_t next_hb;
};
static struct hbmon mon[N + 1];

struct frame {
	uint64_t at;
	uint8_t from, bus;
	uint16_t id;
	uint8_t d[8];
};
static struct frame q[QMAX];
static int qn;
static unsigned long sent_by_msg[8], relayed;

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
	mon[i] = (struct hbmon){.boot = now, .next_hb = now + 1000 * (unsigned)i};
	clients[i] = (struct raft_client){0};
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

/* the mini HB_MON */

static bool quiet(const struct hbmon *m, int bus, uint64_t limit)
{
	uint64_t last = m->last_rx[bus] ? m->last_rx[bus] : m->boot;

	return now - last >= limit;
}

static uint8_t own_health(int j)
{
	const struct hbmon *m = &mon[j];
	bool qa = quiet(m, 0, DEGRADED_US), qb = quiet(m, 1, DEGRADED_US);

	return (uint8_t)((qa && !qb ? RAFT_HEALTH_BIT(0) : 0) | (qb && !qa ? RAFT_HEALTH_BIT(1) : 0));
}

static void build_obs(int j, struct raft_obs *o)
{
	const struct hbmon *m = &mon[j];

	*o = (struct raft_obs){0};
	o->heard_anyone = !quiet(m, 0, DEGRADED_US) || !quiet(m, 1, DEGRADED_US);
	for (int x = 1; x <= N; x++) {
		if (x == j) {
			o->alive[x] = true;
			o->health_fresh[x] = true;
			o->health[x] = own_health(j);
			continue;
		}
		o->alive[x] = m->last_hb[x] && now - m->last_hb[x] < RAFT_NODE_DOWN_MS * 1000u;
		o->health_fresh[x] = m->last_hb[x] && now - m->last_hb[x] < HEALTH_FRESH_US;
		o->health[x] = m->health[x];
	}
}

static void send_hb(int j)
{
	uint8_t d[8] = {mon[j].hb_seq++ & 0x0F, 1, own_health(j)};

	enqueue(j, 0, HB_ID(j), d);
	enqueue(j, 1, HB_ID(j), d);
}

/* a node received a frame: HB_MON, raft, and maybe relay it */
static void receive(int j, const struct frame *f)
{
	struct hbmon *m = &mon[j];
	uint8_t sender = 0;
	bool is_raft = raft_msg_from_id(f->id, &sender) != RAFT_MSG_NONE;

	m->last_rx[f->bus] = now;
	if (f->id >= HB_ID(1) && f->id <= HB_ID(N)) {
		sender = (uint8_t)(f->id - HB_ID(1) + 1);
		m->last_hb[sender] = now;
		m->health[sender] = f->d[2];
	}
	if (is_raft) {
		raft_receive_frame(&nodes[j], f->id, f->d);
	}

	/* relay: raft frames already relayed once are never relayed again */
	if (sender != 0 && !(is_raft && (f->d[0] & RAFT_FLAG_RELAYED))) {
		struct net_state committed, eff;
		struct raft_obs o;
		int to;

		getNetwork(&nodes[j], &committed);
		build_obs(j, &o);
		route_effective_net(&committed, &o, (uint8_t)j, &eff);
		to = route_relay_bus(&eff, (uint8_t)j, sender, f->bus);
		if (to >= 0) {
			uint8_t d[8];

			memcpy(d, f->d, 8);
			if (is_raft) {
				d[0] |= RAFT_FLAG_RELAYED;
			}
			relayed++;
			enqueue(j, to, f->id, d);
		}
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
				receive(j, &f);
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
		deliver();
		for (int i = 1; i <= N; i++) {
			if (!alive[i]) {
				continue;
			}
			if (now >= mon[i].next_hb) {
				mon[i].next_hb += HB_PERIOD_US;
				send_hb(i);
			}
			if (now % TICK == 0) {
				struct raft_obs o;
				bool all = true;

				raft_tick(&nodes[i]);
				build_obs(i, &o);
				raft_client_update(&clients[i], &nodes[i], &o, now);
				for (int x = 1; x <= N; x++) {
					all &= o.alive[x];
				}
				if (all) {
					raft_peers_ready(&nodes[i]);
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
	int expect_links;  /* number of CANLINK entries, -1 = don't check */
	bool no_node_entries; /* link faults only: nobody should be reported down */
};

static const struct scenario scenarios[] = {
	{"s1", "Demo 1: link failure, default mode (no predefined leader)", RAFT_LEADER_RANDOM, 2000,
	 {{1000, CUT, 2, RAFT_BUS_A}}, 1, -1, 0, false, 1, true},
	{"s2", "Demo 2: link failure and recovery, leader setup mode", RAFT_NODE_COCKPIT, 2500,
	 {{500, CUT, 3, RAFT_BUS_A}, {1500, FIX, 3, RAFT_BUS_A}}, 2, 1, 1, true, 2, true},
	{"s3", "Demo 3: leader failure, leader setup mode", RAFT_NODE_COCKPIT, 2000,
	 {{500, KILL, 1, 0}}, 1, 2, -1, false, -1, false},
	{"c4", "Checkoff 4: N2 off A, restore, then off B, restore (no election)", RAFT_NODE_COCKPIT, 3000,
	 {{500, CUT, 2, RAFT_BUS_A}, {1500, FIX, 2, RAFT_BUS_A}, {1600, CUT, 2, RAFT_BUS_B},
	  {2500, FIX, 2, RAFT_BUS_B}}, 4, 1, 1, true, 4, true},
	{"c5", "Checkoff 5: N1 off B, N3 off A (relay through N2, term unchanged)", RAFT_NODE_COCKPIT,
	 2500, {{500, CUT, 1, RAFT_BUS_B}, {1000, CUT, 3, RAFT_BUS_A}}, 2, 1, 1, false, 2, true},
	{"c67", "Checkoff 6-7: Pi down, STM32 leads, Pi back as follower", RAFT_NODE_COCKPIT, 3500,
	 {{500, KILL, 1, 0}, {1500, BOOT, 1, 0}}, 2, 2, -1, false, -1, false},
	{"c9", "Checkoff 9: N2 leads, N2 off A, N3 off B (relay through the Pi)", RAFT_NODE_STEERING,
	 2500, {{500, CUT, 2, RAFT_BUS_A}, {1000, CUT, 3, RAFT_BUS_B}}, 2, 1, 2, false, 2, true},
	{"c10", "Checkoff 10: isolate follower N3, restore (term unchanged)", RAFT_NODE_COCKPIT, 2500,
	 {{500, CUT, 3, RAFT_BUS_A}, {500, CUT, 3, RAFT_BUS_B}, {1500, FIX, 3, RAFT_BUS_A},
	  {1500, FIX, 3, RAFT_BUS_B}}, 4, 1, 1, false, -1, false},
	{"c10L", "Checkoff 10: isolate leader N1, restore (term changes once)", RAFT_NODE_COCKPIT, 2500,
	 {{500, CUT, 1, RAFT_BUS_A}, {500, CUT, 1, RAFT_BUS_B}, {1500, FIX, 1, RAFT_BUS_A},
	  {1500, FIX, 1, RAFT_BUS_B}}, 4, 2, -1, false, -1, false},
};

static void reset(uint8_t leader_mode, uint32_t seed)
{
	now = 0;
	qn = 0;
	relayed = 0;
	memset(sent_by_msg, 0, sizeof(sent_by_msg));
	memset(disk, 0, sizeof(disk));
	opts = (struct raft_opts){.default_leader = leader_mode, .seed = seed};
	for (int i = 1; i <= N; i++) {
		wire[i][0] = wire[i][1] = true;
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

/* all live nodes' getNetwork() should match the real wiring */
static bool network_matches_wires(void)
{
	for (int i = 1; i <= N; i++) {
		struct net_state ns;

		if (!alive[i]) {
			continue;
		}
		getNetwork(&nodes[i], &ns);
		for (int x = 1; x <= N; x++) {
			bool up = alive[x] && (wire[x][0] || wire[x][1]);

			if (ns.node_up[x] != up) {
				printf("  N%d thinks N%d is %s\n", i, x, ns.node_up[x] ? "up" : "down");
				return false;
			}
			for (int b = 0; up && b < 2; b++) {
				if (ns.link_up[x][b] != wire[x][b]) {
					printf("  N%d thinks N%d bus %c is %s\n", i, x, 'A' + b,
					       ns.link_up[x][b] ? "up" : "down");
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

static int count_entries(enum raft_ekind kind)
{
	int c = 0;

	for (int i = 1; i <= N; i++) {
		if (alive[i]) {
			for (uint16_t k = 1; k <= nodes[i].commit_index; k++) {
				c += raft_entry_decode(nodes[i].log[k].entry).kind == kind;
			}
			return c;
		}
	}
	return 0;
}

static bool run_scenario(const struct scenario *sc)
{
	int who = 0, leaders;
	bool ok = true;
	uint8_t term = 0;

	printf("\n=== %s: %s\n", sc->name, sc->what);
	reset(sc->leader_mode, 4); /* seed 4: an STM32 wins under LEADER=random */
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
	if (!network_matches_wires()) {
		printf("  FAIL: getNetwork() doesn't match the wiring\n");
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
	if (sc->expect_links >= 0 && count_entries(RAFT_EK_LINK) != sc->expect_links) {
		printf("  FAIL: %d CANLINK entries, expected %d\n", count_entries(RAFT_EK_LINK),
		       sc->expect_links);
		ok = false;
	}
	if (sc->no_node_entries && count_entries(RAFT_EK_NODE) != 0) {
		printf("  FAIL: a node was reported down, but only links were cut\n");
		ok = false;
	}
	printf("  frames: REF %lu, APPEND %lu, ACK %lu, REQ_VOTE %lu, VOTE %lu, EVENT %lu, REDIRECT %lu, relayed %lu\n",
	       sent_by_msg[RAFT_MSG_REF], sent_by_msg[RAFT_MSG_APPEND],
	       sent_by_msg[RAFT_MSG_APPEND_ACK], sent_by_msg[RAFT_MSG_REQ_VOTE],
	       sent_by_msg[RAFT_MSG_VOTE], sent_by_msg[RAFT_MSG_EVENT], sent_by_msg[RAFT_MSG_REDIRECT],
	       relayed);
	printf("  %s\n", ok ? "PASS" : "FAIL");
	return ok;
}

static void seed_table(void)
{
	printf("LEADER=random: winner per seed (elections start once each node hears the others)\n");
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
		fprintf(stderr, "unknown scenario %s (try s1 s2 s3 c4 c5 c67 c9 c10 c10L seeds)\n", only);
		return 2;
	}
	printf("\n%d/%d scenarios passed\n", ran - failed, ran);
	return failed ? 1 : 0;
}
