/* CAN on both MCP2515s, from lab 3. See canbus.h. */
#include <string.h>
#include <zephyr/sys/atomic.h>

#include "canbus.h"
#include "gtime.h"
#include "raft_msg.h"
#include "tp.h"
#include "zone.h"

K_MSGQ_DEFINE(canbus_rxq, sizeof(struct rx_item), 64, 8);

/*
 * The MCP2515 driver filters in software, max 5 filters. We take everything
 * another node could send, since any node may have to relay it:
 *   0x000-0x07F  CTRL, FAULT and all the raft frames (0x031-0x067)
 *   0x100-0x103  heartbeats
 *   0x200        BLINK
 *   0x300        FORCE
 */
static const struct can_filter filters[] = {
	{.id = 0x000, .mask = 0x780},
	{.id = CAN_ID_HB_N1, .mask = 0x7FC},
	{.id = CAN_ID_BLINK, .mask = CAN_STD_ID_MASK},
	{.id = CAN_ID_FORCE, .mask = CAN_STD_ID_MASK},
};

static const struct device *const devs[BUS_COUNT] = {
	DEVICE_DT_GET(DT_NODELABEL(can_a)),
	DEVICE_DT_GET(DT_NODELABEL(can_b)),
};
static const enum tp tp_rx[BUS_COUNT] = {TP_CAN_RX_A, TP_CAN_RX_B};
static const enum tp tp_tx[BUS_COUNT] = {TP_CAN_TX_A, TP_CAN_TX_B};

/* how long to wait for the (single) MCP2515 TX buffer, see lab 3 notes */
#define TX_WAIT K_MSEC(2)

static bool up[BUS_COUNT];
static atomic_t tx_ok[BUS_COUNT], tx_fail[BUS_COUNT], rx_cnt[BUS_COUNT], rx_drop;

/* runs in the MCP2515 driver thread right after the frame is read */
static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct rx_item it = {.frame = *frame, .t_us = gtime_local_us(), .bus = (uint8_t)(uintptr_t)user_data};

	ARG_UNUSED(dev);
	tp_toggle(tp_rx[it.bus]);
	atomic_inc(&rx_cnt[it.bus]);
	if (k_msgq_put(&canbus_rxq, &it, K_NO_WAIT) != 0) {
		atomic_inc(&rx_drop);
	}
}

static void tx_cb(const struct device *dev, int error, void *user_data)
{
	int bus = (int)(uintptr_t)user_data;

	ARG_UNUSED(dev);
	atomic_inc(error == 0 ? &tx_ok[bus] : &tx_fail[bus]);
}

/* REF finished sending: the "finished" timestamp for time sync (two-step) */
static void ref_tx_cb(const struct device *dev, int error, void *user_data)
{
	uint64_t t = gtime_local_us();
	int bus = (int)((uintptr_t)user_data & 0xFF);
	uint8_t cycle = (uint8_t)((uintptr_t)user_data >> 8);

	ARG_UNUSED(dev);
	if (error == 0) {
		gtime_ref_sent(bus, cycle, t);
		atomic_inc(&tx_ok[bus]);
	} else {
		atomic_inc(&tx_fail[bus]);
	}
}

int canbus_init(void)
{
	int n_up = 0;

	for (int b = 0; b < BUS_COUNT; b++) {
		bool ok = device_is_ready(devs[b]);

		for (int i = 0; ok && i < ARRAY_SIZE(filters); i++) {
			ok = can_add_rx_filter(devs[b], rx_cb, (void *)(uintptr_t)b, &filters[i]) >= 0;
		}
		if (ok) {
			ok = can_start(devs[b]) == 0;
		}
		up[b] = ok;
		n_up += ok;
		printk("CAN %c: %s\n", 'A' + b, ok ? "started" : "NOT running");
	}
	return n_up > 0 ? 0 : -ENODEV;
}

/* On a bus nobody ACKs (jumper pulled) the TX buffer never frees, so don't
 * sit waiting for it: try once and move on. */
static k_timeout_t tx_wait_for(int bus)
{
	uint8_t degraded = zone_health_bits() & (bus == BUS_A ? CAN_HEALTH_A_DEGRADED : CAN_HEALTH_B_DEGRADED);

	return degraded ? K_NO_WAIT : TX_WAIT;
}

static int send_frame(int bus, uint16_t id, const uint8_t d[CAN_DLC], can_tx_callback_t cb,
		      void *user_data)
{
	struct can_frame f = {.id = id, .dlc = CAN_DLC};
	int err;

	if (!up[bus]) {
		return -ENODEV;
	}
	memcpy(f.data, d, CAN_DLC);
	tp_toggle(tp_tx[bus]);
	err = can_send(devs[bus], &f, tx_wait_for(bus), cb, user_data);
	if (err != 0) {
		atomic_inc(&tx_fail[bus]);
	}
	return err;
}

int canbus_send_on(int bus, uint16_t id, const uint8_t d[CAN_DLC])
{
	return send_frame(bus, id, d, tx_cb, (void *)(uintptr_t)bus);
}

int canbus_send_ref_on(int bus, uint16_t id, const uint8_t d[CAN_DLC])
{
	uint8_t out[CAN_DLC];

	memcpy(out, d, CAN_DLC);
	gtime_ref_fill(bus, out);
	return send_frame(bus, id, out, ref_tx_cb, (void *)(uintptr_t)(bus | (out[2] << 8)));
}

void canbus_send_both(uint16_t id, const uint8_t d[CAN_DLC], uint8_t *seq)
{
	canbus_send_on(BUS_A, id, d);
	canbus_send_on(BUS_B, id, d);
	*seq = can_seq_next(*seq);
}

void canbus_get_stats(struct canbus_stats *s)
{
	for (int b = 0; b < BUS_COUNT; b++) {
		s->tx_ok[b] = atomic_get(&tx_ok[b]);
		s->tx_fail[b] = atomic_get(&tx_fail[b]);
		s->rx[b] = atomic_get(&rx_cnt[b]);
		if (!up[b] || can_get_state(devs[b], &s->state[b], NULL) != 0) {
			s->state[b] = CAN_STATE_STOPPED;
		}
	}
	s->rx_drop = atomic_get(&rx_drop);
}
