/* Raft on the Pi, see raftnode.h. */
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "gtime.h"
#include "raftnode.h"
#include "util.h"

static struct raft_node raft;
static struct raft_client client;
static struct canbus *can;
static bool silent;
static struct net_state eff; /* routing table */
static uint64_t next_client_us, boot_us;

static FILE *logf;
static uint16_t shown_upto; /* highest index already shown (before a restart) */

/* port functions */

static uint64_t port_now(void *ctx)
{
	(void)ctx;
	return now_us();
}

static void port_send(void *ctx, uint16_t id, const uint8_t d[8])
{
	uint8_t s;
	enum raft_msg m = raft_msg_from_id(id, &s);

	(void)ctx;
	if (silent && m != RAFT_MSG_EVENT) {
		return; /* failed ourselves: silent, except the EVENT that says why */
	}
	if (m == RAFT_MSG_REF) {
		/* time sync: each bus gets the time our last REF finished on that bus */
		for (int bus = 0; bus < BUS_COUNT; bus++) {
			uint8_t out[8];

			memcpy(out, d, 8);
			gtime_ref_fill(bus, out);
			canbus_send_on(can, bus, id, out);
		}
		return;
	}
	canbus_send(can, id, d);
}

static void port_log(void *ctx, const char *line)
{
	(void)ctx;
	printf("%s\n", line);
}

/* the cockpit "display": every committed entry, once */
static void port_commit(void *ctx, uint16_t index, uint8_t term, uint8_t entry, bool replayed)
{
	char s[24];
	uint64_t g = global_gettime();

	(void)ctx;
	if (index <= shown_upto) {
		return; /* shown before we restarted */
	}
	shown_upto = index;
	raft_entry_str(entry, s, sizeof(s));
	printf("LOG #%u (term %u) %s%s\n", index, term, s, replayed ? "  [replayed]" : "");
	if (logf) {
		fprintf(logf, "#%u term %u %s%s global %llu.%06llu\n", index, term, s,
			replayed ? " [replayed]" : "", (unsigned long long)(g / 1000000u),
			(unsigned long long)(g % 1000000u));
		fflush(logf);
	}
}

static void port_save(void *ctx, uint8_t term, uint8_t voted)
{
	FILE *f = fopen(RAFT_STATE_FILE ".tmp", "w");

	(void)ctx;
	if (!f) {
		perror(RAFT_STATE_FILE);
		return;
	}
	fprintf(f, "%u %u\n", term, voted);
	fclose(f);
	rename(RAFT_STATE_FILE ".tmp", RAFT_STATE_FILE); /* all or nothing */
}

static const struct raft_port port = {
	.now_us = port_now,
	.send = port_send,
	.entry_committed = port_commit,
	.save_state = port_save,
	.log = port_log,
};

/* startup */

static bool load_state(struct raft_saved *saved)
{
	FILE *f = fopen(RAFT_STATE_FILE, "r");
	unsigned t, v;
	bool ok = false;

	if (f) {
		ok = fscanf(f, "%u %u", &t, &v) == 2;
		fclose(f);
	}
	if (ok) {
		saved->term = (uint8_t)t;
		saved->voted_for = (uint8_t)v;
	}
	return ok;
}

/* highest "#<index>" line in the log file */
static uint16_t last_shown_index(void)
{
	FILE *f = fopen(RAFT_LOG_FILE, "r");
	char line[160];
	unsigned idx, best = 0;

	if (!f) {
		return 0;
	}
	while (fgets(line, sizeof(line), f)) {
		if (sscanf(line, "#%u", &idx) == 1 && idx > best) {
			best = idx;
		}
	}
	fclose(f);
	return (uint16_t)best;
}

int raftnode_init(struct canbus *c, bool warm)
{
	struct raft_saved saved = {0};
	bool have = false;

	can = c;
	boot_us = now_us();
	if (warm) {
		have = load_state(&saved);
		shown_upto = last_shown_index();
		logf = fopen(RAFT_LOG_FILE, "a");
		printf("raft: rejoining a running cluster (term %u from %s, %u entries already shown)\n",
		       have ? saved.term : 0, RAFT_STATE_FILE, shown_upto);
	} else {
		remove(RAFT_STATE_FILE);
		logf = fopen(RAFT_LOG_FILE, "w");
		printf("raft: cold start (no leader heard), starting fresh\n");
	}
	if (!logf) {
		perror(RAFT_LOG_FILE);
	}
	raft_init(&raft, RAFT_NODE_COCKPIT, &port, NULL, have ? &saved : NULL);
	printf("raft: node 1, default leader %u, seed %u, log %u entries\n", RAFT_DEFAULT_LEADER,
	       RAFT_SEED, RAFT_LOG_SIZE);
	return 0;
}

void raftnode_close(void)
{
	if (logf) {
		fclose(logf);
		logf = NULL;
	}
}

/* running */

void raftnode_set_silent(bool s)
{
	silent = s;
}

void raftnode_frame(uint16_t id, const uint8_t d[8])
{
	raft_receive_frame(&raft, id, d);
}

void raftnode_tick(const struct raft_obs *o, uint64_t now)
{
	raft_tick(&raft);
	if (now < next_client_us) {
		return;
	}
	next_client_us = now + MS(CLIENT_MS);

	struct net_state committed;
	bool all = true;

	raft_client_update(&client, &raft, o, now);
	for (int x = 1; x <= RAFT_N; x++) {
		all &= o->alive[x];
	}
	/* LEADER=random: start the election clock once everyone's here */
	if (all || now - boot_us > MS(PEERS_READY_MAX_MS)) {
		raft_peers_ready(&raft);
	}
	getNetwork(&raft, &committed);
	route_effective_net(&committed, o, RAFT_NODE_COCKPIT, &eff);
	gtime_set_leader(raft_is_leader(&raft));
}

int raftnode_route(uint8_t sender, int from_bus)
{
	return route_relay_bus(&eff, RAFT_NODE_COCKPIT, sender, from_bus);
}

void raftnode_report(uint8_t entry)
{
	raft_report_change(&raft, entry);
}

bool raftnode_is_leader(void)
{
	return raft_is_leader(&raft);
}

void raftnode_print_status(void)
{
	struct net_state net;
	char s[96];

	getNetwork(&raft, &net);
	net_str(&net, s, sizeof(s));
	printf("raft: %s term %u leader N%u commit %u/%u | getNetwork: %s\n", raft_role_str(raft.role),
	       raft_term(&raft), raft_leader_id(&raft), raft_commit_index(&raft), raft_last_index(&raft), s);
}

void raftnode_print_log(void)
{
	char s[24];

	printf("log: %u entries, %u committed\n", raft_last_index(&raft), raft_commit_index(&raft));
	for (uint16_t i = 1; i <= raft_last_index(&raft); i++) {
		struct raft_log_entry e;

		raft_log_get(&raft, i, &e);
		printf("  #%u term %u %s%s\n", i, e.term, raft_entry_str(e.entry, s, sizeof(s)),
		       i <= raft_commit_index(&raft) ? "" : " (not committed)");
	}
}
