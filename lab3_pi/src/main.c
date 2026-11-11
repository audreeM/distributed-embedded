/*
 * Cockpit zone N1 (Raspberry Pi, Linux) for Lab 3.
 *
 * One thread, one poll() loop, like Lab 2. Inputs: the wheel stream on UDP
 * :8000, can0 (bus A), can1 (bus B), a 1 ms timerfd and the keyboard.
 * Template tasks from the Lab 3 task table:
 *   UDP_RX    handle_udp()          wheel packet in, toggles UDP_RX
 *   CTRL_TX   send_ctrl()           0x010 on both buses, toggles CMD_TX
 *   BLINK_TX  blinker.c + tick()    0x200 on change and on every lamp edge
 *   HB_TX     tick()                0x100 every 20 ms unless LOCAL_FAIL
 *   HB_MON    handle_can() + health.c
 *   FORCE_RX  handle_can()          0x300 in, UDP force byte out every 50 ms
 *
 * Key b (then Enter): the next CTRL carries throttle 150 (out of range).
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
#include "gpio_tp.h"
#include "health.h"
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

	/* force feedback: latest FORCE value, forwarded to the proxy */
	int8_t force;
	bool have_force;
	uint64_t next_force_us;
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

static void send_blink(struct ctx *x, uint64_t now)
{
	struct can_blink b = {.mode = x->blink.mode, .phase = x->blink.phase};
	uint8_t d[CAN_DLC];

	can_pack_blink(d, next_seq(x, TX_BLINK), &b);
	canbus_send(&x->can, CAN_ID_BLINK, d);
	x->blink.last_sent_us = now;
}

/* Sent even when the zone has just gone silent: it is the zone's last word. */
static void send_fault(struct ctx *x, uint8_t reason, uint64_t now)
{
	struct can_fault f = {.zone = CAN_ZONE_N1, .reason = reason};
	uint8_t d[CAN_DLC];

	can_pack_fault(d, next_seq(x, TX_FAULT), &f);
	canbus_send(&x->can, CAN_ID_FAULT, d);
	x->last_fault_us = now;
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

/* Everything waiting on one bus. Duplicates still count for bus health. */
static void handle_can(struct ctx *x, int bus)
{
	uint16_t id;
	uint8_t d[CAN_DLC], dlc;

	while (canbus_recv(&x->can, bus, &id, d, &dlc) > 0) {
		uint64_t now = now_us();
		uint32_t ms = us_to_ms32(now);
		uint8_t zone = can_hb_zone(id);

		health_on_frame(&x->health, bus, id, now);
		if (dlc != CAN_DLC) {
			continue;
		}
		if (zone != 0) {
			struct can_hb hb;

			if (can_seq_accept(&x->rx_hb[zone], can_seq(d), ms) &&
			    can_unpack_hb(d, &hb) == CAN_OK) {
				health_on_heartbeat(&x->health, zone, &hb);
			}
		} else if (id == CAN_ID_FORCE) {
			struct can_force f;

			if (can_seq_accept(&x->rx_force, can_seq(d), ms) &&
			    can_unpack_force(d, &f) == CAN_OK) {
				x->force = f.force;
				x->have_force = true;
			}
		} else if (id == CAN_ID_FAULT) {
			struct can_fault f;

			/* Unpack first: the seq counter belongs to the zone in the payload. */
			if (can_unpack_fault(d, &f) == CAN_OK &&
			    can_seq_accept(&x->rx_fault[f.zone], can_seq(d), ms)) {
				zone_on_fault(&x->zone, &f, now);
			}
		}
	}
}

/* ------------------------------------------------------------- 1 ms tick */

static void print_line(struct ctx *x, uint64_t now)
{
	const struct health *h = &x->health;
	const char *st = x->zone.state == ZONE_NORMAL ? "NORMAL" :
			 x->zone.state == ZONE_ERROR ? "ERROR" : "LOCAL_FAIL";

	printf("%-10s | UDP %s | A tx %u fail %u rx %u | B tx %u fail %u rx %u | "
	       "N2 %s N3 %s | DEG %c%c | blink %u/%u | force %+d\n",
	       st, udp_fresh(x, now) ? "ok" : "STALE",
	       x->can.tx_ok[BUS_A], x->can.tx_fail[BUS_A], x->can.rx[BUS_A],
	       x->can.tx_ok[BUS_B], x->can.tx_fail[BUS_B], x->can.rx[BUS_B],
	       h->zone_failed[CAN_ZONE_N2] ? "FAILED" : h->zone_alive[CAN_ZONE_N2] ? "ok" : "--",
	       h->zone_failed[CAN_ZONE_N3] ? "FAILED" : h->zone_alive[CAN_ZONE_N3] ? "ok" : "--",
	       h->degraded[BUS_A] ? 'A' : '-', h->degraded[BUS_B] ? 'B' : '-',
	       x->blink.mode, x->blink.phase, x->force);
	fflush(stdout);
}

static void tick(struct ctx *x, int tfd)
{
	static uint64_t next_print;
	uint64_t expirations;
	uint64_t now = now_us();

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

	/* BLINK_TX: the blinker decides, main sends */
	struct blinker_inputs bin = {
		.left_btn = fresh && x->wheel.left,
		.right_btn = fresh && x->wheel.right,
		.steer = x->wheel.ctrl.steer,
		.hazard = x->zone.state != ZONE_NORMAL,
	};

	if (blinker_update(&x->blink, &bin, now) && tx) {
		send_blink(x, now);
	}

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

	if (now >= next_print) {
		next_print = now + MS(PRINT_MS);
		print_line(x, now);
	}
}

static void handle_keyboard(struct ctx *x, struct pollfd *pfd)
{
	char line[64];

	if (!fgets(line, sizeof(line), stdin)) {
		pfd->fd = -1; /* stdin closed */
		return;
	}
	if (line[0] == 'b') {
		x->inject_bad = true;
		printf(">>> next CTRL carries throttle 150 (out of range)\n");
	} else if (line[0] == 'q') {
		running = 0;
	}
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

	static const int tp_pins[] = {TP_UDP_RX, TP_CMD_TX, TP_CAN_TX_A, TP_CAN_TX_B,
				      LED_DEG_A, LED_DEG_B, LED_ERR};
	tp_init(tp_pins, (int)(sizeof(tp_pins) / sizeof(tp_pins[0])));

	if (canbus_open(&x.can) < 0) {
		return 1;
	}

	uint64_t now = now_us();

	health_init(&x.health, now);
	zone_init(&x.zone);
	blinker_init(&x.blink);
	x.next_hb_us = now;
	x.next_force_us = now;

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

	tp_release();
	canbus_close(&x.can);
	close(x.udp);
	close(tfd);
	return 0;
}
