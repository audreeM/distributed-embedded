/*
 * Cockpit zone (Raspberry Pi, Linux).
 *
 * Replaces proxy_receiver: receives the wheel stream on UDP :8000, scales it
 * to the link command (throttle/brake 0..100 %, steer -100..+100, buttons),
 * and sends it to the STM32 over the serial link:
 *   - immediately on every UDP state update, and
 *   - again whenever CMD_REFRESH_MS pass without a send,
 *   - but not at all once the wheel stream is UDP_STALE_MS old, so a dead
 *     proxy looks like a dead link to the STM32 and it fails safe.
 * Also parses the STM32's 20 ms status frames and prints them.
 *
 * Keys (then Enter): b = send out-of-range commands for 1 s,
 *                    c = send one frame with a corrupted CRC, q = quit.
 */
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "gpio_tp.h"
#include "link_proto.h"
#include "serial.h"
#include "state.h"

/* Fail the build (instead of silently misparsing) if the struct ever changes size. */
_Static_assert(sizeof(DIJOYSTATE2_t) == WHEEL_PKT_LEN - 4, "DIJOYSTATE2 layout");

/* All timestamps in this file are microseconds; MS() converts from ms. */
#define MS(x) ((uint64_t)(x) * 1000u)

/* Cleared by Ctrl-C / SIGTERM. sig_atomic_t is the only type a signal
 * handler may safely write.
 */
static volatile sig_atomic_t running = 1;

/*
 * All program state. Everything runs in one thread (a single poll() loop),
 * so no locking is needed anywhere in this file.
 */
struct ctx {
	/* --- command path (Pi -> STM32) --- */
	int ser;                 /* serial port fd */
	uint8_t seq;             /* SEQ for the next command frame (wraps at 255) */
	struct lp_cmd last;      /* last scaled wheel state; resent on refresh */
	bool have_udp;           /* at least one wheel packet received */
	uint64_t last_udp_us;    /* when the last wheel packet arrived */
	uint64_t last_send_us;   /* when the last command was written */
	uint64_t inject_until_us; /* 'b' key: send out-of-range frames until then */

	/* --- counters shown on the console --- */
	uint32_t udp_pkts, udp_lost, udp_bad_len, cmds_sent, tx_errs;
	uint32_t last_counter;   /* proxy's packet counter, to detect lost packets */
	bool have_counter;

	/* --- status path (STM32 -> Pi) --- */
	struct lp_parser parser; /* shared framing parser from link_proto.h */
	struct lp_status st;     /* most recent STATUS frame */
	struct lp_currents cur;  /* most recent CURRENTS frame */
	bool have_status;
	uint32_t status_frames, status_rx_err;
	uint64_t last_status_us;
	/* status inter-arrival stats: current 1 s window and the last full one */
	uint64_t win_start_us, win_min, win_max, win_sum;
	uint32_t win_n;
	double shown_min, shown_avg, shown_max;
	uint32_t shown_n;
};

static void on_signal(int sig)
{
	(void)sig;
	running = 0;
}

/* Monotonic clock: unaffected by NTP / wall-clock changes, so time deltas
 * are always correct.
 */
static uint64_t now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* Linear map raw [from..to] -> [lo..hi], clamped. Works for reversed ranges. */
static int map_axis(long raw, long from, long to, int lo, int hi)
{
	if (to == from) {
		return lo;
	}
	double t = (double)(raw - from) / (double)(to - from);

	if (t < 0.0) {
		t = 0.0;
	} else if (t > 1.0) {
		t = 1.0;
	}
	return (int)lround(lo + t * (hi - lo));
}

/* DirectInput marks a pressed button by setting the high bit (0x80) of its
 * byte. idx < 0 means "not configured in config.h" and always reads released.
 */
static bool button(const DIJOYSTATE2_t *s, int idx)
{
	return idx >= 0 && idx < 128 && (s->rgbButtons[idx] & 0x80);
}

/*
 * Raw wheel counts -> link command in engineering units. The conversion is
 * done here, on the Pi, so the STM32 never has to know which proxy (Mac or
 * Windows) or which wheel calibration is upstream.
 */
static struct lp_cmd scale(const DIJOYSTATE2_t *s)
{
	struct lp_cmd c = {0};
	int steer;

	/* Map each half separately, so a wheel whose center is not exactly
	 * halfway between the ends still gives 0 at center and +/-100 at the
	 * stops. Both halves are linear, so the mapping stays monotonic.
	 */
	if (s->lX < STEER_RAW_CENTER) {
		steer = map_axis(s->lX, STEER_RAW_LEFT, STEER_RAW_CENTER, -100, 0);
	} else {
		steer = map_axis(s->lX, STEER_RAW_CENTER, STEER_RAW_RIGHT, 0, 100);
	}
	if (abs(steer) <= STEER_DEADBAND) {
		steer = 0;
	}
	c.steer = (int8_t)steer;

	/* Pedals: released -> 0 %, fully pressed -> 100 %. The deadband stops
	 * pedal noise from creeping the motor or dragging the brake (the STM32
	 * treats any brake > 0 as "brake").
	 */
	int thr = map_axis(s->lY, THR_RAW_RELEASED, THR_RAW_PRESSED, 0, 100);
	int brk = map_axis(s->lRz, BRK_RAW_RELEASED, BRK_RAW_PRESSED, 0, 100);

	c.throttle = (uint8_t)(thr < THR_DEADBAND ? 0 : thr);
	c.brake = (uint8_t)(brk < BRK_DEADBAND ? 0 : brk);

	bool left = button(s, BTN_IDX_LEFT);
	bool right = button(s, BTN_IDX_RIGHT);

	/* Both at once is invalid on the link; send neither. */
	if (left != right) {
		c.buttons |= left ? LP_BTN_LEFT : LP_BTN_RIGHT;
	}
	if (button(s, BTN_IDX_TEST)) {
		c.buttons |= LP_BTN_TEST;
	}
	return c;
}

/*
 * Encode and write one command frame from x->last.
 * corrupt = true flips the CRC so we can show the STM32 dropping it.
 */
static void send_cmd(struct ctx *x, bool corrupt)
{
	struct lp_cmd c = x->last;
	uint8_t buf[LP_FRAME_MAX];

	if (now_us() < x->inject_until_us) {
		c.throttle = 150; /* CRC-valid but out of range */
	}
	size_t n = lp_encode_cmd(buf, x->seq++, &c);

	if (corrupt) {
		buf[n - 1] ^= 0xFF;
	}
	/* CMD_TX edge right before the bytes go to the UART driver, so
	 * CMD_TX -> CMD_RX on the scope = serial wire time + STM32 parse time.
	 */
	tp_toggle(TP_CMD_TX);
	/* 11 bytes always fit in the kernel tty buffer, so this non-blocking
	 * write either sends the whole frame or fails; a short write counts as
	 * an error.
	 */
	if (write(x->ser, buf, n) != (ssize_t)n) {
		x->tx_errs++;
	}
	x->last_send_us = now_us();
	x->cmds_sent++;
}

/* One wheel packet from the proxy: 4-byte LE counter + DIJOYSTATE2. */
static void on_udp(struct ctx *x, int sock)
{
	uint8_t buf[512];
	ssize_t n = recv(sock, buf, sizeof(buf), 0);

	if (n < 0) {
		return;
	}
	/* Anything that is not exactly one wheel packet is ignored. */
	if (n != WHEEL_PKT_LEN) {
		x->udp_bad_len++;
		return;
	}
	tp_toggle(TP_UDP_RX);

	/* The proxy increments its counter on every packet, so a jump means
	 * UDP packets were lost (normal now and then on Wi-Fi).
	 */
	uint32_t counter = buf[0] | (buf[1] << 8) | (buf[2] << 16) | ((uint32_t)buf[3] << 24);

	if (x->have_counter && counter > x->last_counter + 1) {
		x->udp_lost += counter - x->last_counter - 1;
	}
	x->last_counter = counter;
	x->have_counter = true;
	x->udp_pkts++;

	/* memcpy instead of a pointer cast: buf+4 is not aligned for ints. */
	DIJOYSTATE2_t state;

	memcpy(&state, buf + 4, sizeof(state));
	x->last = scale(&state);
	x->last_udp_us = now_us();
	x->have_udp = true;
	send_cmd(x, false); /* forward right away: this is the latency path */
}

/*
 * Bytes from the STM32. The serial port has no framing of its own, so every
 * byte goes through the shared parser, which finds frame boundaries and
 * checks the CRC.
 */
static void on_serial(struct ctx *x)
{
	uint8_t buf[256];
	ssize_t n;

	while ((n = read(x->ser, buf, sizeof(buf))) > 0) {
		uint64_t now = now_us();

		for (ssize_t i = 0; i < n; i++) {
			enum lp_result r = lp_parse_byte(&x->parser, buf[i], (uint32_t)now);

			if (r == LP_OK && x->parser.frame.type == LP_TYPE_STATUS) {
				lp_decode_status(&x->parser.frame, &x->st);
				/* Track the gap between status frames: this is
				 * how we show the 20 ms +/- 10 % heartbeat.
				 */
				if (x->have_status) {
					uint64_t dt = now - x->last_status_us;

					if (x->win_n == 0 || dt < x->win_min) {
						x->win_min = dt;
					}
					if (dt > x->win_max) {
						x->win_max = dt;
					}
					x->win_sum += dt;
					x->win_n++;
				}
				x->have_status = true;
				x->last_status_us = now;
				x->status_frames++;
			} else if (r == LP_OK && x->parser.frame.type == LP_TYPE_CURRENTS) {
				lp_decode_currents(&x->parser.frame, &x->cur);
			} else if (r == LP_ERR_CRC || r == LP_ERR_LEN || r == LP_ERR_GAP) {
				x->status_rx_err++;
			}
		}
	}
}

/* One console line: what we are sending, and what the STM32 reports back. */
static void print_line(struct ctx *x, uint64_t now)
{
	bool fresh = x->have_udp && now - x->last_udp_us < MS(UDP_STALE_MS);

	printf("UDP %6u (lost %u) %s| CMD %6u thr=%3u brk=%3u str=%+4d btn=%c%c%c%s | ",
	       x->udp_pkts, x->udp_lost, fresh ? "" : "STALE ", x->cmds_sent,
	       x->last.throttle, x->last.brake, x->last.steer,
	       (x->last.buttons & LP_BTN_LEFT) ? 'L' : '-',
	       (x->last.buttons & LP_BTN_RIGHT) ? 'R' : '-',
	       (x->last.buttons & LP_BTN_TEST) ? 'T' : '-',
	       now < x->inject_until_us ? " INJECTING-BAD" : "");

	/* No status for 100 ms: the STM32 is off, not flashed, or the RX wire
	 * is out.
	 */
	if (!x->have_status || now - x->last_status_us > MS(STATUS_SILENT_MS)) {
		printf("STM SILENT\n");
	} else {
		printf("STM %s f=0x%02x ack=%3u crcErr=%u rngErr=%u I=%u/%u/%u mA "
		       "hb=%.1f/%.1f/%.1f ms (n=%u)\n",
		       x->st.state == LP_STATE_NORMAL ? "NORMAL" : "ERROR ", x->st.faults,
		       x->st.last_seq, x->st.crc_err, x->st.range_err, x->cur.i_motor_l_ma,
		       x->cur.i_motor_r_ma, x->cur.i_servo_ma, x->shown_min, x->shown_avg,
		       x->shown_max, x->shown_n);
	}
	fflush(stdout);
}

/*
 * Runs every TICK_MS (5 ms). Handles everything that is time-driven rather
 * than event-driven: command refresh, heartbeat stats, console printing.
 */
static void on_tick(struct ctx *x, int tfd)
{
	static uint64_t next_print;
	uint64_t expirations;
	uint64_t now = now_us();

	/* Reading the timerfd acknowledges the tick (it returns how many ticks
	 * passed, which we don't need).
	 */
	if (read(tfd, &expirations, sizeof(expirations)) < 0) {
		return;
	}

	bool fresh = x->have_udp && now - x->last_udp_us < MS(UDP_STALE_MS);
	bool injecting = now < x->inject_until_us;

	/* Refresh: if the proxy is slower than 20 ms (e.g. --interval-ms 50) or
	 * a packet was lost, resend the last command so the STM32 still gets
	 * one at least every 20-25 ms. If the wheel stream is stale we send
	 * NOTHING: the STM32 times out and fails safe, which is what we want
	 * when the proxy has died.
	 */
	if ((fresh || injecting) && now - x->last_send_us >= MS(CMD_REFRESH_MS)) {
		send_cmd(x, false);
	}

	/* Every second, freeze the heartbeat stats for display and start a
	 * new window.
	 */
	if (now - x->win_start_us >= MS(1000)) {
		x->shown_n = x->win_n;
		x->shown_min = x->win_n ? x->win_min / 1000.0 : 0;
		x->shown_max = x->win_n ? x->win_max / 1000.0 : 0;
		x->shown_avg = x->win_n ? (double)x->win_sum / x->win_n / 1000.0 : 0;
		x->win_n = 0;
		x->win_min = x->win_max = x->win_sum = 0;
		x->win_start_us = now;
	}

	if (now >= next_print) {
		next_print = now + MS(PRINT_MS);
		print_line(x, now);
	}
}

/* Keyboard commands for fault injection during checkoff. */
static void on_stdin(struct ctx *x, struct pollfd *pfd)
{
	char line[64];

	if (!fgets(line, sizeof(line), stdin)) {
		pfd->fd = -1; /* stdin closed (e.g. running under nohup) */
		return;
	}
	switch (line[0]) {
	case 'b':
		x->inject_until_us = now_us() + MS(INJECT_BAD_MS);
		printf(">>> injecting out-of-range throttle for %d ms\n", INJECT_BAD_MS);
		break;
	case 'c':
		send_cmd(x, true);
		printf(">>> sent one corrupt-CRC frame\n");
		break;
	case 'q':
		running = 0;
		break;
	default:
		printf("keys: b = out-of-range for 1 s, c = corrupt CRC frame, q = quit\n");
		break;
	}
}

static void usage(const char *argv0)
{
	fprintf(stderr, "usage: %s [-d serial_dev] [-r]\n"
			"  -d  serial device (default %s)\n"
			"  -r  SCHED_FIFO priority 80 + mlockall (run with sudo)\n",
		argv0, SERIAL_DEV);
}

int main(int argc, char **argv)
{
	const char *dev = SERIAL_DEV;
	bool realtime = false;
	int opt;

	while ((opt = getopt(argc, argv, "d:rh")) != -1) {
		switch (opt) {
		case 'd':
			dev = optarg;
			break;
		case 'r':
			realtime = true;
			break;
		default:
			usage(argv[0]);
			return opt == 'h' ? 0 : 1;
		}
	}

	/* Optional: realtime priority so other Linux processes can't delay the
	 * UDP -> serial path, and locked memory so page faults can't either.
	 * Use it when taking timing captures.
	 */
	if (realtime) {
		struct sched_param sp = {.sched_priority = 80};

		if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0) {
			perror("sched_setscheduler (need sudo?)");
		}
		if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
			perror("mlockall");
		}
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	static const int tp_pins[] = {TP_UDP_RX, TP_CMD_TX};

	tp_init(tp_pins, 2);

	struct ctx x = {0};

	x.ser = serial_open(dev);
	if (x.ser < 0) {
		return 1;
	}
	lp_parser_init(&x.parser, PARSER_GAP_US);
	x.win_start_us = now_us();

	/* Bind to INADDR_ANY (every interface) instead of a hard-coded IP like
	 * receiver.c did, so a changed campus IP address doesn't break us.
	 */
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(UDP_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};

	if (sock < 0 || bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("UDP bind (is proxy_receiver or another copy still running?)");
		return 1;
	}

	/* timerfd: a periodic timer that shows up as a readable fd, so it fits
	 * into the same poll() loop as the sockets (no second thread).
	 */
	int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
	struct itimerspec its = {
		.it_interval = {.tv_nsec = TICK_MS * 1000000L},
		.it_value = {.tv_nsec = TICK_MS * 1000000L},
	};

	if (tfd < 0 || timerfd_settime(tfd, 0, &its, NULL) < 0) {
		perror("timerfd");
		return 1;
	}

	printf("cockpit: UDP :%d -> %s @115200. Test points GPIO%d=UDP_RX GPIO%d=CMD_TX\n",
	       UDP_PORT, dev, TP_UDP_RX, TP_CMD_TX);
	if (BTN_IDX_LEFT < 0 || BTN_IDX_RIGHT < 0 || BTN_IDX_TEST < 0) {
		printf("warning: some BTN_IDX_* in config.h are not set yet\n");
	}
	printf("keys: b = out-of-range for 1 s, c = corrupt CRC frame, q = quit\n");

	struct pollfd pfd[4] = {
		{.fd = sock, .events = POLLIN},
		{.fd = x.ser, .events = POLLIN},
		{.fd = tfd, .events = POLLIN},
		{.fd = STDIN_FILENO, .events = POLLIN},
	};

	/* Main loop: sleep until any fd is ready, then handle it. */
	while (running) {
		if (poll(pfd, 4, -1) < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("poll");
			break;
		}
		/* UDP first: it is the latency-critical path. */
		if (pfd[0].revents & POLLIN) {
			on_udp(&x, sock);
		}
		if (pfd[1].revents & POLLIN) {
			on_serial(&x);
		}
		if (pfd[2].revents & POLLIN) {
			on_tick(&x, tfd);
		}
		if (pfd[3].revents & (POLLIN | POLLHUP)) {
			on_stdin(&x, &pfd[3]);
		}
	}

	tp_release();
	close(x.ser);
	close(sock);
	close(tfd);
	return 0;
}
