/*
 * Raft on the STM32: one thread owns the raft_node (raft isn't thread safe).
 *
 *   - raft frames from dispatch come in through raft_q
 *   - raft_tick() every RAFT_TICK_MS
 *   - every CLIENT_MS: build the raft_obs from HB_MON, run the client (reports
 *     link/node changes to the leader), and publish the routing table that
 *     dispatch uses to decide what to relay
 *   - LEADER LED, and tells time sync whether we're the leader
 *
 * Console (serial terminal at 115200):
 *   e   report a test EVENT from this node (checkoff 5: "an EVENT from N2")
 *   s   print raft status
 *   l   print the committed log
 */
#include <stdio.h>
#include <string.h>
#include <zephyr/console/console.h>
#include <zephyr/kernel.h>

#include "canbus.h"
#include "config.h"
#include "gtime.h"
#include "raft_client.h"
#include "raft_task.h"
#include "tp.h"
#include "zone.h"

struct raft_rx {
	uint16_t id;
	uint8_t d[8];
};

K_MSGQ_DEFINE(raft_q, sizeof(struct raft_rx), 48, 4);
K_MSGQ_DEFINE(report_q, sizeof(uint8_t), 8, 1);

static struct raft_node raft; /* ~60 KB, mostly the log */
static struct raft_client client;

/* published for other threads */
static struct k_spinlock pub_lock;
static struct net_state pub_eff;   /* routing table */
static struct raft_status pub_status;
static bool pub_failure;
static K_MUTEX_DEFINE(log_lock);   /* raft_task_print_log reads the log */

/* port functions */

static uint64_t port_now(void *ctx)
{
	ARG_UNUSED(ctx);
	return gtime_local_us();
}

static void port_send(void *ctx, uint16_t id, const uint8_t d[8])
{
	uint8_t s;
	enum raft_msg m = raft_msg_from_id(id, &s);

	ARG_UNUSED(ctx);
	/* a node that failed itself (test button) goes silent, raft included.
	 * Its own EVENT still goes out, like the FAULT that goes with it. */
	if (zone_state() == ZONE_LOCAL_FAIL && m != RAFT_MSG_EVENT) {
		return;
	}
	for (int bus = 0; bus < BUS_COUNT; bus++) {
		if (m == RAFT_MSG_REF) {
			canbus_send_ref_on(bus, id, d);
		} else {
			canbus_send_on(bus, id, d);
		}
	}
}

static void port_log(void *ctx, const char *line)
{
	ARG_UNUSED(ctx);
	printk("%s\n", line);
}

static const struct raft_port port = {
	.now_us = port_now,
	.send = port_send,
	.log = port_log,
	/* commits are printed by raft's debug output; no disk to save term/vote to */
};

/* called from other threads */

void raft_task_post(uint16_t id, const uint8_t d[8])
{
	struct raft_rx rx = {.id = id};

	memcpy(rx.d, d, 8);
	(void)k_msgq_put(&raft_q, &rx, K_NO_WAIT);
}

void raft_task_report(uint8_t entry)
{
	(void)k_msgq_put(&report_q, &entry, K_NO_WAIT);
}

int raft_task_route(uint8_t sender, int from_bus)
{
	k_spinlock_key_t key = k_spin_lock(&pub_lock);
	struct net_state eff = pub_eff;

	k_spin_unlock(&pub_lock, key);
	return route_relay_bus(&eff, NODE_ID, sender, from_bus);
}

bool raft_task_failure_mode(void)
{
	k_spinlock_key_t key = k_spin_lock(&pub_lock);
	bool f = pub_failure;

	k_spin_unlock(&pub_lock, key);
	return f;
}

void raft_task_status(struct raft_status *s)
{
	k_spinlock_key_t key = k_spin_lock(&pub_lock);

	*s = pub_status;
	k_spin_unlock(&pub_lock, key);
}

void raft_task_print_log(void)
{
	char s[24];
	uint16_t last, commit;

	k_mutex_lock(&log_lock, K_FOREVER);
	last = raft.last_index;
	commit = raft.commit_index;
	k_mutex_unlock(&log_lock);
	printk("log: %u entries, %u committed\n", last, commit);
	for (uint16_t i = 1; i <= last; i++) {
		struct raft_log_entry e;
		bool ok;

		/* one entry at a time: printing takes ms, don't hold up the raft thread */
		k_mutex_lock(&log_lock, K_FOREVER);
		ok = raft_log_get(&raft, i, &e);
		commit = raft.commit_index;
		k_mutex_unlock(&log_lock);
		if (!ok) {
			break;
		}
		printk("  #%u term %u %s%s\n", i, e.term, raft_entry_str(e.entry, s, sizeof(s)),
		       i <= commit ? "" : " (not committed)");
	}
}

/* the thread */

static void publish(const struct raft_obs *o)
{
	struct net_state committed, eff;
	struct raft_status st = {
		.role = raft.role, .term = raft.term, .leader = raft.leader,
		.last_index = raft.last_index, .commit_index = raft.commit_index,
	};

	getNetwork(&raft, &committed);
	route_effective_net(&committed, o, NODE_ID, &eff);
	st.net = committed;

	k_spinlock_key_t key = k_spin_lock(&pub_lock);

	pub_eff = eff;
	pub_status = st;
	pub_failure = !committed.cockpit_ok;
	k_spin_unlock(&pub_lock, key);

	tp_set(TP_LEADER, raft_is_leader(&raft));
	gtime_set_leader(raft_is_leader(&raft));
}

static void raft_thread(void *p1, void *p2, void *p3)
{
	struct raft_rx rx;
	struct raft_obs o;
	uint8_t entry;
	uint64_t next_tick = 0, next_client = 0, boot = gtime_local_us();

	k_mutex_lock(&log_lock, K_FOREVER);
	raft_init(&raft, NODE_ID, &port, NULL, NULL);
	k_mutex_unlock(&log_lock);
	printk("raft: node %u, default leader %u, seed %u, log %u entries (%u bytes)\n", NODE_ID,
	       RAFT_DEFAULT_LEADER, RAFT_SEED, RAFT_LOG_SIZE, (unsigned)sizeof(raft));

	for (;;) {
		bool got = k_msgq_get(&raft_q, &rx, K_MSEC(1)) == 0;
		uint64_t now;

		k_mutex_lock(&log_lock, K_FOREVER);
		/* a few frames per pass: raft's debug prints take ~5 ms a line, and
		 * the watchdog needs to hear from this thread every 300 ms */
		for (int k = 0; got && k < 4; k++) {
			raft_receive_frame(&raft, rx.id, rx.d);
			got = k < 3 && k_msgq_get(&raft_q, &rx, K_NO_WAIT) == 0;
		}
		while (k_msgq_get(&report_q, &entry, K_NO_WAIT) == 0) {
			raft_report_change(&raft, entry);
		}
		now = gtime_local_us();
		if (now >= next_tick) {
			next_tick = now + RAFT_TICK_MS * 1000u;
			raft_tick(&raft);
		}
		if (now >= next_client) {
			bool all = true;

			next_client = now + CLIENT_MS * 1000u;
			zone_obs(&o);
			raft_client_update(&client, &raft, &o, now);
			for (int x = 1; x <= RAFT_N; x++) {
				all &= o.alive[x];
			}
			/* LEADER=random: start the election clock once everyone's here */
			if (all || now - boot > PEERS_READY_MAX_MS * 1000ull) {
				raft_peers_ready(&raft);
			}
			publish(&o);
		}
		k_mutex_unlock(&log_lock);
		zone_wdt_checkin(WDT_RAFT);
	}
}

K_THREAD_DEFINE(raft_tid, STACK_SZ_RAFT, raft_thread, NULL, NULL, NULL, PRIO_RAFT, 0, SYS_FOREVER_MS);

static void print_status(void)
{
	struct raft_status st;
	char net[96];

	raft_task_status(&st);
	net_str(&st.net, net, sizeof(net));
	printk("raft: %s term %u leader N%u commit %u/%u | getNetwork: %s\n", raft_role_str(st.role),
	       st.term, st.leader, st.commit_index, st.last_index, net);
}

static void console_thread(void *p1, void *p2, void *p3)
{
	console_init();
	for (;;) {
		int c = console_getchar();

		switch (c) {
		case 'e':
			printk(">>> reporting EVENT_N%u_0x0F (test)\n", NODE_ID);
			raft_task_report(raft_e_event(NODE_ID, 0xF));
			break;
		case 's':
			print_status();
			break;
		case 'l':
			raft_task_print_log();
			break;
		default:
			break;
		}
	}
}

K_THREAD_DEFINE(console_tid, STACK_SZ, console_thread, NULL, NULL, NULL, PRIO_CONSOLE, 0,
		SYS_FOREVER_MS);

int raft_task_init(void)
{
	k_thread_start(raft_tid);
	k_thread_start(console_tid);
	return 0;
}

void raft_task_print_status(void)
{
	print_status();
}
