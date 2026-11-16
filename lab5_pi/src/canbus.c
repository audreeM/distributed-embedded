/*
 * SocketCAN plumbing. One raw socket per interface does both TX and RX.
 *
 * Lab 5: CAN_RAW_RECV_OWN_MSGS makes the socket get its own frames back.
 * The driver echoes a frame when the controller says it finished sending
 * (TX done interrupt), and SO_TIMESTAMPNS stamps it then, which is exactly
 * the "REF finished sending" time two-step time sync needs. The kernel marks
 * those echoes with MSG_CONFIRM so we can tell them apart.
 */
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "canbus.h"
#include "config.h"
#include "gpio_tp.h"
#include "util.h"

// opens the interface for can
static int open_one(const char *ifname)
{
	int s = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
	unsigned int idx = if_nametoindex(ifname);
	struct sockaddr_can addr = {.can_family = AF_CAN, .can_ifindex = (int)idx};

	// check for can interface init errrors!
	if (s < 0 || idx == 0 || bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		fprintf(stderr, "%s: can't open (run can_up.sh first)\n", ifname);
		if (s >= 0) {
			close(s);
		}
		return -1;
	}
	int on = 1;

	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on)) < 0 ||
	    setsockopt(s, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) < 0) {
		perror("CAN socket options (time sync won't work)");
	}
	return s;
}

// inits the can bus
int canbus_open(struct canbus *cb)
{
	memset(cb, 0, sizeof(*cb));
	cb->fd[BUS_A] = open_one(CAN_IF_A);
	cb->fd[BUS_B] = open_one(CAN_IF_B);

	// returns if can bus init was successful
	return (cb->fd[BUS_A] >= 0 || cb->fd[BUS_B] >= 0) ? 0 : -1;
}

// sends data on can bus
unsigned canbus_send(struct canbus *cb, uint16_t id, const uint8_t data[CAN_DLC])
{
	struct can_frame f = {.can_id = id, .can_dlc = CAN_DLC};
	unsigned ok = 0;

	memcpy(f.data, data, CAN_DLC);
	for (int bus = 0; bus < BUS_COUNT; bus++) {
		tp_toggle(bus == BUS_A ? TP_CAN_TX_A : TP_CAN_TX_B);
		/* Non-blocking: if nobody on this bus ACKs (jumper pulled) the
		 * queue fills and the write fails, and the other bus is not held
		 * up waiting for it (R4.7).
		 */
		if (cb->fd[bus] >= 0 && write(cb->fd[bus], &f, sizeof(f)) == (ssize_t)sizeof(f)) {
			cb->tx_ok[bus]++;
			ok |= 1u << bus;
		} else {
			cb->tx_fail[bus]++;
		}
	}
	return ok;
}

int canbus_send_on(struct canbus *cb, int bus, uint16_t id, const uint8_t data[CAN_DLC])
{
	struct can_frame f = {.can_id = id, .can_dlc = CAN_DLC};

	memcpy(f.data, data, CAN_DLC);
	tp_toggle(bus == BUS_A ? TP_CAN_TX_A : TP_CAN_TX_B);
	if (cb->fd[bus] >= 0 && write(cb->fd[bus], &f, sizeof(f)) == (ssize_t)sizeof(f)) {
		cb->tx_ok[bus]++;
		return 0;
	}
	cb->tx_fail[bus]++;
	return -1;
}

/* kernel timestamps are CLOCK_REALTIME; everything here runs on CLOCK_MONOTONIC */
static uint64_t realtime_to_mono_us(const struct timespec *rt)
{
	struct timespec now_rt;
	uint64_t mono = now_us();

	clock_gettime(CLOCK_REALTIME, &now_rt);
	int64_t age_ns = ((int64_t)now_rt.tv_sec - rt->tv_sec) * 1000000000LL + (now_rt.tv_nsec - rt->tv_nsec);

	return mono - (uint64_t)(age_ns / 1000);
}

int canbus_recv(struct canbus *cb, int bus, uint16_t *id, uint8_t data[CAN_DLC], uint8_t *dlc,
		uint64_t *t_us, bool *own)
{
	struct can_frame f;
	char ctrl[CMSG_SPACE(sizeof(struct timespec))];
	struct iovec iov = {.iov_base = &f, .iov_len = sizeof(f)};
	struct msghdr msg = {.msg_iov = &iov, .msg_iovlen = 1, .msg_control = ctrl,
			     .msg_controllen = sizeof(ctrl)};

	if (cb->fd[bus] < 0) {
		return 0;
	}
	ssize_t n = recvmsg(cb->fd[bus], &msg, 0);

	if (n < 0) {
		return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
	}
	if (n != (ssize_t)sizeof(f) || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))) {
		return 0; /* not a standard data frame: nothing in the catalog */
	}
	*t_us = now_us();
	for (struct cmsghdr *c = CMSG_FIRSTHDR(&msg); c != NULL; c = CMSG_NXTHDR(&msg, c)) {
		if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
			struct timespec ts;

			memcpy(&ts, CMSG_DATA(c), sizeof(ts));
			*t_us = realtime_to_mono_us(&ts);
		}
	}
	*own = (msg.msg_flags & MSG_CONFIRM) != 0;
	*id = (uint16_t)(f.can_id & CAN_SFF_MASK);
	*dlc = f.can_dlc;
	memset(data, 0, CAN_DLC);
	memcpy(data, f.data, f.can_dlc <= CAN_DLC ? f.can_dlc : CAN_DLC);
	if (!*own) {
		cb->rx[bus]++;
	}
	return 1;
}

void canbus_close(struct canbus *cb)
{
	for (int bus = 0; bus < BUS_COUNT; bus++) {
		if (cb->fd[bus] >= 0) {
			close(cb->fd[bus]);
		}
	}
}
