/*
 * Cockpit zone N1 (Raspberry Pi, Linux) for Lab 5.
 *
 * Lab 3's single poll() loop (wheel UDP, can0, can1, 1 ms timer, keyboard)
 * plus:
 *   - raft (raftnode.c): election, log, client, routing table
 *   - relaying any node's frames when the routing table says the Pi is the
 *     only bridge between the buses (checkoff 9)
 *   - time sync (gtime.c): REF timestamps from the kernel, global time
 *   - PPS / HZ on their own realtime thread (edges.c)
 *   - LEADER / SYNC LEDs, REF_RX test point
 *
 * Keys (then Enter):
 *   b  next CTRL carries throttle 150 (out of range)
 *   e  report a test EVENT from the cockpit
 *   s  raft status        l  print the raft log        q  quit
 */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include "blinker.h"
#include "can_catalog.h"
#include "canbus.h"
#include "config.h"
#include "edges.h"
#include "gpio_tp.h"
#include "gtime.h"
#include "health.h"
#include "raft_msg.h"
#include "raftnode.h"
#include "util.h"
#include "wheel.h"
#include "zone.h"

/* One 4-bit sequence counter per ID the cockpit sends (R4.4). */
enum { TX_CTRL, TX_FAULT, TX_HB, TX_BLINK, TX_COUNT };

static volatile sig_atomic_t running = 1;

struct ctx {
	struct canbus can;
	struct health health;
	struct zone zone;
	struct blinker blink;

	/* wheel stream */
	int udp;
	struct sockaddr_in proxy; /* where the last wheel packet came from */
	struct wheel wheel;
	bool have_wheel;
	uint64_t last_udp_us;

	/* transmit side */
	uint8_t tx_seq[TX_COUNT];
	uint64_t last_ctrl_us, next_hb_us, last_fault_us;
	bool inject_bad; /* 'b': next CTRL carries an out-of-range value */

	/* receive side: one duplicate filter per (sender, ID) */
	struct can_seq_filter rx_hb[ZONE_SLOTS];
	struct can_seq_filter rx_fault[ZONE_SLOTS];
	struct can_seq_filter rx_force;
	struct can_seq_filter rx_ref[ZONE_SLOTS]; /* REF_RX test point only */

	/* force feedback: latest FORCE value, forwarded to the proxy */
	int8_t force;
	bool have_force;
	uint64_t next_force_us;

	uint32_t relayed;
};

static void handle_signal(int sig)
{
	(void)sig;
	running = 0;
}

static uint8_t next_seq(struct ctx *x, int which)
{
	uint8_t seq = x->tx_seq[which];

	x->tx_seq[which] = can_seq_next(seq);
	return seq;
}

static bool udp_fresh(const struct ctx *x, uint64_t now)
{
	return x->have_wheel && now - x->last_udp_us < MS(UDP_STALE_MS);
}

/* ---------------------------------------------------------------- transmit */

static void send_ctrl(struct ctx *x, uint64_t now)
{
	struct can_ctrl c = x->wheel.ctrl;
	uint8_t d[CAN_DLC];

	if (x->inject_bad) {
		c.throttle = 150;
		x->inject_bad = false;
	}
	can_pack_ctrl(d, next_seq(x, TX_CTRL), &c);
	canbus_send(&x->can, CAN_ID_CTRL, d);
	/* After both writes: UDP_RX -> CMD_TX on the scope is the Pi stage. */
	tp_toggle(TP_CMD_TX);
	x->last_ctrl_us = now;
}

static void send_hb(struct ctx *x)
{
	struct can_hb hb = {.state = zone_wire_state(&x->zone), .health = health_bits(&x->health)};
	uint8_t d[CAN_DLC];

	can_pack_hb(d, next_seq(x, TX_HB), &hb);
	canbus_send(&x->can, CAN_ID_HB_N1, d);
}

static void send_blink(struct ctx *x, uint64_t global_now)
{
	struct can_blink b = {
		.mode = x->blink.mode, .phase = x->blink.phase,
		.start_ms = (uint32_t)(x->blink.start_us / 1000u),
	};
	uint8_t d[CAN_DLC];

	can_pack_blink(d, next_seq(x, TX_BLINK), &b);
	canbus_send(&x->can, CAN_ID_BLINK, d);
	x->blink.last_sent_us = global_now;
}

/* Sent even when the zone has just gone silent: it is the zone's last word. */
static void send_fault(struct ctx *x, uint8_t reason, uint64_t now)
{
	struct can_fault f = {.zone = CAN_ZONE_N1, .reason = reason};
	uint8_t d[CAN_DLC];

	can_pack_fault(d, next_seq(x, TX_FAULT), &f);
	canbus_send(&x->can, CAN_ID_FAULT, d);
	x->last_fault_us = now;
	/* lab 5: the fault also goes in the raft log, as EVENT_N1_<reason> */
	raftnode_report(raft_e_event(CAN_ZONE_N1, reason));
}

static void send_force_udp(struct ctx *x)
{
	struct sockaddr_in to = x->proxy;
	int8_t f = x->force;

	to.sin_port = htons(FORCE_UDP_PORT);
	(void)sendto(x->udp, &f, 1, 0, (struct sockaddr *)&to, sizeof(to));
}

/* ----------------------------------------------------------------- receive */

/* One wheel packet from the proxy: the start of the 2 ms brake chain. */
static void handle_udp(struct ctx *x)
{
	uint8_t buf[512];
	struct sockaddr_in from;
	socklen_t fromlen = sizeof(from);
	ssize_t n = recvfrom(x->udp, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromlen);

	if (n < 0 || !wheel_parse(buf, (size_t)n, &x->wheel)) {
		return;
	}
	tp_toggle(TP_UDP_RX);
	x->have_wheel = true;
	x->last_udp_us = now_us();
	x->proxy = from;

	/* R4.4: CTRL on every UDP state update, while the zone transmits. */
	if (zone_transmitting(&x->zone)) {
		send_ctrl(x, x->last_udp_us);
	}
}

/* Part 5: copy the frame to the other bus if the routing table says the Pi is the bridge. */
static void maybe_relay(struct ctx *x, int bus, uint16_t id, const uint8_t d[CAN_DLC], uint8_t sender,
			bool is_raft)
{
	int to;
	uint8_t out[CAN_DLC];

	if (sender == 0 || !zone_transmitting(&x->zone) || (is_raft && (d[0] & RAFT_FLAG_RELAYED))) {
		return;
	}
	to = raftnode_route(sender, bus);
	if (to < 0) {
		return;
	}
	memcpy(out, d, CAN_DLC);
	if (is_raft) {
		out[0] |= RAFT_FLAG_RELAYED;
	}
	if (raft_id_is_ref(id)) {
		gtime_ref_fill(to, out); /* our time for the last REF we put on that bus */
	}
	if (canbus_send_on(&x->can, to, id, out) == 0) {
		x->relayed++;
	}
}

/* Everything waiting on one bus. Duplicates still count for bus health. */
static void handle_can(struct ctx *x, int bus)
{
	uint16_t id;
	uint8_t d[CAN_DLC], dlc;
	uint64_t t;
	bool own;

	while (canbus_recv(&x->can, bus, &id, d, &dlc, &t, &own) > 0) {
		uint64_t now = now_us();
		uint32_t ms = us_to_ms32(now);
		uint8_t zone = can_hb_zone(id);
		uint8_t sender = 0;
		enum raft_msg m = raft_msg_from_id(id, &sender);

		if (own) {
			/* our own frame, done sending: the REF timestamp for time sync */
			if (m == RAFT_MSG_REF && dlc == CAN_DLC) {
				gtime_ref_sent(bus, d[2], t);
			}
			continue;
		}
		health_on_frame(&x->health, bus, id, now);
		if (dlc != CAN_DLC) {
			continue;
		}
		if (m != RAFT_MSG_NONE) {
			if (sender == CAN_ZONE_N1) {
				continue;
			}
			if (m == RAFT_MSG_REF) {
				gtime_ref_received(bus, sender, d, t);
				if (can_seq_accept(&x->rx_ref[sender], d[0] & RAFT_SEQ_MASK, ms)) {
					tp_toggle(TP_REF_RX);
				}
			}
			raftnode_frame(id, d);
			maybe_relay(x, bus, id, d, sender, true);
			continue;
		}
		if (zone != 0) {
			struct can_hb hb;

			if (zone != CAN_ZONE_N1 && can_seq_accept(&x->rx_hb[zone], can_seq(d), ms) &&
			    can_unpack_hb(d, &hb) == CAN_OK) {
				health_on_heartbeat(&x->health, zone, &hb, now);
				maybe_relay(x, bus, id, d, zone, false);
			}
		} else if (id == CAN_ID_FORCE) {
			struct can_force f;

			if (can_seq_accept(&x->rx_force, can_seq(d), ms) && can_unpack_force(d, &f) == CAN_OK) {
				x->force = f.force;
				x->have_force = true;
				maybe_relay(x, bus, id, d, CAN_ZONE_N2, false);
			}
		} else if (id == CAN_ID_FAULT) {
			struct can_fault f;

			/* Unpack first: the seq counter belongs to the zone in the payload. */
			if (can_unpack_fault(d, &f) == CAN_OK && f.zone != CAN_ZONE_N1 &&
			    can_seq_accept(&x->rx_fault[f.zone], can_seq(d), ms)) {
				zone_on_fault(&x->zone, &f, now);
				maybe_relay(x, bus, id, d, f.zone, false);
			}
		}
	}
}

/* ------------------------------------------------------------- 1 ms tick */

static void print_status(struct ctx *x, uint64_t now)
{
	const struct health *h = &x->health;
	const char *st = x->zone.state == ZONE_NORMAL ? "NORMAL" :
			 x->zone.state == ZONE_ERROR ? "ERROR" : "LOCAL_FAIL";
	struct gtime_stats gs;
	char drift[32];

	printf("%-10s | UDP %s | A tx %u fail %u rx %u | B tx %u fail %u rx %u | "
	       "N2 %s N3 %s | DEG %c%c | blink %u/%u | force %+d | relayed %u\n",
	       st, udp_fresh(x, now) ? "ok" : "STALE",
	       x->can.tx_ok[BUS_A], x->can.tx_fail[BUS_A], x->can.rx[BUS_A],
	       x->can.tx_ok[BUS_B], x->can.tx_fail[BUS_B], x->can.rx[BUS_B],
	       h->zone_failed[CAN_ZONE_N2] ? "FAILED" : h->zone_alive[CAN_ZONE_N2] ? "ok" : "--",
	       h->zone_failed[CAN_ZONE_N3] ? "FAILED" : h->zone_alive[CAN_ZONE_N3] ? "ok" : "--",
	       h->degraded[BUS_A] ? 'A' : '-', h->degraded[BUS_B] ? 'B' : '-',
	       x->blink.mode, x->blink.phase, x->force, x->relayed);
	raftnode_print_status();
	gtime_get_stats(&gs);
	gtime_fmt_drift(drift, sizeof(drift), gs.drift_ppb);
	/* part 6.2: the drift correction currently applied */
	printf("sync: %s drift %s, last correction %+lld us, %u samples (%u rejected)\n",
	       gs.leader ? "time master," : gs.synced ? "synced," : "NOT synced,", drift,
	       (long long)gs.last_error_us, gs.samples, gs.rejected);
	fflush(stdout);
}

/* part 6.4: one line per PPS edge */
static void print_pps(const struct pps_record *r)
{
	char drift[32];

	gtime_fmt_drift(drift, sizeof(drift), r->drift_ppb);
	printf("PPS global %llu.%06llu s | local %llu.%06llu s | drift %s | last correction %+lld us | %s\n",
	       (unsigned long long)(r->global_us / 1000000u), (unsigned long long)(r->global_us % 1000000u),
	       (unsigned long long)(r->local_us / 1000000u), (unsigned long long)(r->local_us % 1000000u),
	       drift, (long long)r->last_error_us,
	       r->leader ? "LEADER (time master)" : r->synced ? "synced" : "NOT synced");
	fflush(stdout);
}

static void tick(struct ctx *x, int tfd)
{
	static uint64_t next_print;
	uint64_t expirations;
	uint64_t now = now_us();
	uint64_t g = global_gettime();

	if (read(tfd, &expirations, sizeof(expirations)) < 0) {
		return;
	}
	bool fresh = udp_fresh(x, now);

	/* HB_MON: timestamps -> degraded / failed / isolated */
	health_update(&x->health, now);

	/* Zone state machine (R3) */
	struct zone_inputs zin = {
		.udp_fresh = fresh,
		.test_btn = fresh && x->wheel.test,
		.peers_alive = x->health.zone_alive[CAN_ZONE_N2] && x->health.zone_alive[CAN_ZONE_N3],
		.peer_failed = x->health.zone_failed[CAN_ZONE_N2] || x->health.zone_failed[CAN_ZONE_N3],
		.isolated = x->health.isolated,
	};

	zone_update(&x->zone, &zin, now);
	if (x->zone.fault_to_send != 0 && now - x->last_fault_us >= MS(FAULT_MIN_GAP_MS)) {
		send_fault(x, x->zone.fault_to_send, now);
		x->zone.fault_to_send = 0;
	}
	bool tx = zone_transmitting(&x->zone);

	/* raft: silent along with everything else when the zone failed itself */
	struct raft_obs obs;

	raftnode_set_silent(!tx);
	health_obs(&x->health, now, &obs);
	raftnode_tick(&obs, now);

	/* BLINK_TX: the blinker decides on the global time grid, main sends */
	struct blinker_inputs bin = {
		.left_btn = fresh && x->wheel.left,
		.right_btn = fresh && x->wheel.right,
		.steer = x->wheel.ctrl.steer,
		.hazard = x->zone.state != ZONE_NORMAL,
	};

	if (blinker_update(&x->blink, &bin, g) && tx) {
		send_blink(x, g);
	}
	edges_set_pattern(x->blink.mode, x->blink.start_us, blinker_half_us(x->blink.mode));

	/* HB_TX every 20 ms on a fixed grid, so the period does not drift. */
	if (now >= x->next_hb_us) {
		if (tx) {
			send_hb(x);
		}
		x->next_hb_us += MS(HB_PERIOD_MS);
		if (x->next_hb_us <= now) {
			x->next_hb_us = now + MS(HB_PERIOD_MS);
		}
	}

	/* R4.4: CTRL at least every 50 ms. Stops with a stale wheel stream. */
	if (fresh && tx && now - x->last_ctrl_us >= MS(CTRL_REFRESH_MS)) {
		send_ctrl(x, now);
	}

	/* FORCE_RX -> proxy every 50 ms (R2.3) */
	if (now >= x->next_force_us) {
		x->next_force_us = now + MS(FORCE_UDP_MS);
		if (x->have_force && x->have_wheel) {
			send_force_udp(x);
		}
	}

	tp_set(LED_DEG_A, x->health.degraded[BUS_A]);
	tp_set(LED_DEG_B, x->health.degraded[BUS_B]);
	tp_set(LED_ERR, x->zone.state != ZONE_NORMAL);
	tp_set(LED_LEADER, raftnode_is_leader());
	tp_set(LED_SYNC, gtime_synced());

	struct pps_record pps;

	if (edges_take_pps(&pps)) {
		print_pps(&pps);
	}
	if (now >= next_print) {
		next_print = now + MS(PRINT_MS);
		print_status(x, now);
	}
}

static void handle_keyboard(struct ctx *x, struct pollfd *pfd)
{
	char line[64];

	if (!fgets(line, sizeof(line), stdin)) {
		pfd->fd = -1; /* stdin closed */
		return;
	}
	switch (line[0]) {
	case 'b':
		x->inject_bad = true;
		printf(">>> next CTRL carries throttle 150 (out of range)\n");
		break;
	case 'e':
		printf(">>> reporting EVENT_N1_0x0F (test)\n");
		raftnode_report(raft_e_event(CAN_ZONE_N1, 0xF));
		break;
	case 's':
		raftnode_print_status();
		break;
	case 'l':
		raftnode_print_log();
		break;
	case 'q':
		running = 0;
		break;
	default:
		break;
	}
}

/*
 * Before starting raft: listen for PROBE_MS. If some other node is already
 * sending REF, a cluster is running and the Pi is rejoining after a crash
 * or power-off (checkoff 7): restore term/vote and come back as a follower.
 * If it's quiet, everyone is starting up together: start fresh, which makes
 * the Pi the leader in term 1 when LEADER=cockpit.
 */
static bool probe_for_leader(struct ctx *x)
{
	uint64_t end = now_us() + MS(PROBE_MS);
	struct pollfd pfd[BUS_COUNT] = {
		{.fd = x->can.fd[BUS_A], .events = POLLIN},
		{.fd = x->can.fd[BUS_B], .events = POLLIN},
	};

	while (now_us() < end) {
		int left_ms = (int)((end - now_us()) / 1000) + 1;

		if (poll(pfd, BUS_COUNT, left_ms) <= 0) {
			continue;
		}
		for (int bus = 0; bus < BUS_COUNT; bus++) {
			uint16_t id;
			uint8_t d[CAN_DLC], dlc, sender;
			uint64_t t;
			bool own;

			while ((pfd[bus].revents & POLLIN) &&
			       canbus_recv(&x->can, bus, &id, d, &dlc, &t, &own) > 0) {
				if (!own && raft_msg_from_id(id, &sender) == RAFT_MSG_REF) {
					return true;
				}
			}
		}
	}
	return false;
}

/* -------------------------------------------------------------------- main */

int main(void)
{
	static struct ctx x; /* zeroed */

	/* SCHED_FIFO keeps UDP_RX -> CMD_TX short and steady. Needs sudo. */
	struct sched_param sp = {.sched_priority = RT_PRIORITY};

	if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0) {
		perror("sched_setscheduler (run with sudo for realtime priority)");
	}
	signal(SIGINT, handle_signal);
	signal(SIGTERM, handle_signal);

	static const int tp_pins[] = {TP_UDP_RX, TP_CMD_TX, TP_CAN_TX_A, TP_CAN_TX_B, LED_DEG_A, LED_DEG_B,
				      LED_ERR, LED_LEADER, LED_SYNC, TP_REF_RX, TP_PPS, TP_HZ};
	tp_init(tp_pins, (int)(sizeof(tp_pins) / sizeof(tp_pins[0])));

	if (canbus_open(&x.can) < 0) {
		return 1;
	}

	printf("Lab 5 N1 cockpit: listening %d ms for a running cluster...\n", PROBE_MS);
	raftnode_init(&x.can, probe_for_leader(&x));

	uint64_t now = now_us();

	health_init(&x.health, now);
	zone_init(&x.zone);
	blinker_init(&x.blink);
	x.next_hb_us = now;
	x.next_force_us = now;
	edges_start();

	x.udp = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(UDP_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};

	if (x.udp < 0 || bind(x.udp, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("UDP bind (is another copy still running?)");
		return 1;
	}

	int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
	struct itimerspec its = {
		.it_interval = {.tv_nsec = TICK_MS * 1000000L},
		.it_value = {.tv_nsec = TICK_MS * 1000000L},
	};

	if (tfd < 0 || timerfd_settime(tfd, 0, &its, NULL) < 0) {
		perror("timerfd");
		return 1;
	}

	enum { FD_UDP, FD_CAN_A, FD_CAN_B, FD_TIMER, FD_KEYBOARD, FD_COUNT };
	struct pollfd pfd[FD_COUNT] = {
		[FD_UDP] = {.fd = x.udp, .events = POLLIN},
		[FD_CAN_A] = {.fd = x.can.fd[BUS_A], .events = POLLIN},
		[FD_CAN_B] = {.fd = x.can.fd[BUS_B], .events = POLLIN},
		[FD_TIMER] = {.fd = tfd, .events = POLLIN},
		[FD_KEYBOARD] = {.fd = STDIN_FILENO, .events = POLLIN},
	};

	while (running) {
		if (poll(pfd, FD_COUNT, -1) < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("poll");
			break;
		}
		/* UDP first: it is the latency-critical path. */
		if (pfd[FD_UDP].revents & POLLIN) {
			handle_udp(&x);
		}
		if (pfd[FD_CAN_A].revents & POLLIN) {
			handle_can(&x, BUS_A);
		}
		if (pfd[FD_CAN_B].revents & POLLIN) {
			handle_can(&x, BUS_B);
		}
		if (pfd[FD_TIMER].revents & POLLIN) {
			tick(&x, tfd);
		}
		if (pfd[FD_KEYBOARD].revents & (POLLIN | POLLHUP)) {
			handle_keyboard(&x, &pfd[FD_KEYBOARD]);
		}
	}

	edges_stop();
	raftnode_close();
	tp_release();
	canbus_close(&x.can);
	close(x.udp);
	close(tfd);
	return 0;
}
