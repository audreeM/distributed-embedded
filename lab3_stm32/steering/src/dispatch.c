#include <zephyr/kernel.h>

#include "blink.h"
#include "canbus.h"
#include "config.h"
#include "relay.h"
#include "steer.h"
#include "zone.h"

/* One duplicate filter per (sender, ID) we receive, starting zeroed. */
static struct can_seq_filter rx_ctrl, rx_blink;
static struct can_seq_filter rx_hb[CAN_ZONE_N3 + 1], rx_fault[CAN_ZONE_N3 + 1];

static void dispatch_thread(void *p1, void *p2, void *p3)
{
	struct rx_item it;

	for (;;) {
		k_msgq_get(&canbus_rxq, &it, K_FOREVER);

		const uint8_t *d = it.frame.data;
		uint16_t id = it.frame.id;
		uint32_t now = k_uptime_get_32();
		struct can_seq_filter *filt = NULL;
		uint8_t sender = 0;
		struct can_fault f;
		bool fault_ok = false;

		if (it.frame.dlc != CAN_DLC) {
			zone_on_frame(it.bus, 0, false);
			continue; /* catalog: anything but 8 bytes is broken */
		}

		/* Who sent it, and which duplicate filter it belongs to. */
		switch (id) {
		case CAN_ID_CTRL:
			sender = CAN_ZONE_N1;
			filt = &rx_ctrl;
			break;
		case CAN_ID_BLINK:
			sender = CAN_ZONE_N1;
			filt = &rx_blink;
			break;
		case CAN_ID_HB_N1:
		case CAN_ID_HB_N3:
			sender = can_hb_zone(id);
			filt = &rx_hb[sender];
			break;
		case CAN_ID_FAULT:
			/* Unpack first: the counter belongs to the zone in the payload. */
			fault_ok = can_unpack_fault(d, &f) == CAN_OK && f.zone != CAN_ZONE_N2;
			if (fault_ok) {
				sender = f.zone;
				filt = &rx_fault[sender];
			}
			break;
		default:
			break;
		}

		zone_on_frame(it.bus, sender, can_hb_zone(id) != 0);
		if (filt == NULL || !can_seq_accept(filt, can_seq(d), now)) {
			continue; /* the copy from the other bus, an old leftover, or not ours */
		}

		int to_bus;

		if (zone_relay_to(sender, it.bus, &to_bus)) {
			relay_queue(&it.frame, to_bus);
		}

		switch (id) {
		case CAN_ID_CTRL: {
			struct can_ctrl c;
			enum can_check r = can_unpack_ctrl(d, &c);

			if (r == CAN_OK) {
				steer_on_ctrl(&c);
			}
			zone_on_ctrl(r); /* bad CTRL -> ERROR + FAULT */
			break;
		}
		case CAN_ID_BLINK: {
			struct can_blink b;

			if (can_unpack_blink(d, &b) == CAN_OK) {
				blink_on_msg(&b);
			}
			break;
		}
		case CAN_ID_FAULT:
			zone_on_fault(&f);
			break;
		default:
			break; /* heartbeats: zone_on_frame() above is all HB_MON needs */
		}
	}
}

K_THREAD_DEFINE(dispatch_tid, STACK_SZ, dispatch_thread, NULL, NULL, NULL, PRIO_DISPATCH, 0,
		SYS_FOREVER_MS);

int dispatch_init(void)
{
	k_thread_start(dispatch_tid);
	return 0;
}
