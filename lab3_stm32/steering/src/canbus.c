#include <string.h>
#include <zephyr/sys/atomic.h>

#include "canbus.h"
#include "tp.h"

K_MSGQ_DEFINE(canbus_rxq, sizeof(struct rx_item), 32, 4);

static const uint16_t rx_ids[] = {
	CAN_ID_CTRL, CAN_ID_FAULT, CAN_ID_HB_N1, CAN_ID_HB_N3, CAN_ID_BLINK,
};

static const struct device *const devs[BUS_COUNT] = {
	DEVICE_DT_GET(DT_NODELABEL(can_a)),
	DEVICE_DT_GET(DT_NODELABEL(can_b)),
};
static const enum tp tp_rx[BUS_COUNT] = { TP_CAN_RX_A, TP_CAN_RX_B };
static const enum tp tp_tx[BUS_COUNT] = { TP_CAN_TX_A, TP_CAN_TX_B };

static bool up[BUS_COUNT];
static atomic_t tx_ok[BUS_COUNT], tx_fail[BUS_COUNT], rx_cnt[BUS_COUNT], rx_drop;


static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct rx_item it = { .frame = *frame, .bus = (uint8_t)(uintptr_t)user_data };

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

int canbus_init(void)
{
	int n_up = 0;

	for (int b = 0; b < BUS_COUNT; b++) {
		bool ok = device_is_ready(devs[b]);

		for (int i = 0; ok && i < ARRAY_SIZE(rx_ids); i++) {
			const struct can_filter f = {
				.id = rx_ids[i], .mask = CAN_STD_ID_MASK, .flags = 0,
			};

			ok = can_add_rx_filter(devs[b], rx_cb, (void *)(uintptr_t)b, &f) >= 0;
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

int canbus_send_on(int bus, uint16_t id, const uint8_t d[CAN_DLC])
{
	struct can_frame f = { .id = id, .dlc = CAN_DLC };
	int err;

	if (!up[bus]) {
		return -ENODEV;
	}
	memcpy(f.data, d, CAN_DLC);
	tp_toggle(tp_tx[bus]);
	/* K_NO_WAIT + callback: never blocks, so a dead bus can't hold up the other. */
	err = can_send(devs[bus], &f, K_NO_WAIT, tx_cb, (void *)(uintptr_t)bus);
	if (err != 0) {
		atomic_inc(&tx_fail[bus]);
	}
	return err;
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
