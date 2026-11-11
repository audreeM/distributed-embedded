/*
 * SocketCAN plumbing. One raw socket per interface does both TX and RX; a
 * socket never receives its own frames, so the cockpit does not see its own
 * CTRL / HB_N1 / BLINK (candump does, because it is a different socket).
 */
#include <errno.h>
#include <linux/can.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "canbus.h"
#include "config.h"
#include "gpio_tp.h"

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

int canbus_recv(struct canbus *cb, int bus, uint16_t *id, uint8_t data[CAN_DLC], uint8_t *dlc)
{
	struct can_frame f;

	if (cb->fd[bus] < 0) {
		return 0;
	}
	ssize_t n = read(cb->fd[bus], &f, sizeof(f));

	if (n < 0) {
		return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
	}
	if (n != (ssize_t)sizeof(f) || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))) {
		return 0; /* not a standard data frame: nothing in the catalog */
	}
	*id = (uint16_t)(f.can_id & CAN_SFF_MASK);
	*dlc = f.can_dlc;
	memset(data, 0, CAN_DLC);
	memcpy(data, f.data, f.can_dlc <= CAN_DLC ? f.can_dlc : CAN_DLC);
	cb->rx[bus]++;
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
