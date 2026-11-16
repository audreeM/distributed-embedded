/*
 * Dispatch: takes every received frame and hands it to the right place.
 * Lab 3 plus:
 *   - raft frames go to the raft thread, REF also to time sync
 *   - relaying uses the routing table from the raft log (part 5) instead of
 *     lab 3's "haven't heard the sender on the other bus" rule
 *   - heartbeat payloads go to HB_MON (each node's own link health)
 */
#include <string.h>
#include <zephyr/kernel.h>

#include "canbus.h"
#include "config.h"
#include "edges.h"
#include "gtime.h"
#include "node.h"
#include "raft_msg.h"
#include "raft_task.h"
#include "relay.h"
#include "tp.h"
#include "zone.h"

static struct can_seq_filter rx_ctrl, rx_blink, rx_force;
static struct can_seq_filter rx_hb[CAN_ZONE_N3 + 1], rx_fault[CAN_ZONE_N3 + 1];
static struct can_seq_filter rx_ref[RAFT_N + 1]; /* only for the REF_RX test point */

static void maybe_relay(const struct rx_item *it, uint8_t sender, bool is_raft)
{
	int to;

	/* a raft frame that was already relayed once never gets relayed again */
	if (is_raft && (it->frame.data[0] & RAFT_FLAG_RELAYED)) {
		return;
	}
	to = raft_task_route(sender, it->bus);
	if (to >= 0) {
		struct can_frame f = it->frame;

		if (is_raft) {
			f.data[0] |= RAFT_FLAG_RELAYED;
		}
		relay_queue(&f, to);
	}
}

static void handle_raft(const struct rx_item *it, enum raft_msg m, uint8_t sender)
{
	const uint8_t *d = it->frame.data;

	zone_on_frame(it->bus, sender, false);
	if (sender == NODE_ID) {
		return;
	}
	if (m == RAFT_MSG_REF) {
		/* every copy: time sync pairs REFs per bus */
		gtime_ref_received(it->bus, sender, d, it->t_us);
		if (can_seq_accept(&rx_ref[sender], d[0] & RAFT_SEQ_MASK, (uint32_t)(it->t_us / 1000))) {
			tp_toggle(TP_REF_RX);
		}
	}
	raft_task_post(it->frame.id, d);
	maybe_relay(it, sender, true);
}

static void dispatch_thread(void *p1, void *p2, void *p3)
{
	struct rx_item it;

	for (;;) {
		k_msgq_get(&canbus_rxq, &it, K_FOREVER);

		const uint8_t *d = it.frame.data;
		uint16_t id = it.frame.id;
		uint32_t now = (uint32_t)(it.t_us / 1000);
		struct can_seq_filter *filt = NULL;
		uint8_t sender = 0;
		struct can_fault f;
		enum raft_msg m;

		if (it.frame.dlc != CAN_DLC) {
			zone_on_frame(it.bus, 0, false);
			continue;
		}
		m = raft_msg_from_id(id, &sender);
		if (m != RAFT_MSG_NONE) {
			handle_raft(&it, m, sender);
			continue;
		}

		switch (id) {
		case CAN_ID_CTRL:
			sender = CAN_ZONE_N1;
			filt = &rx_ctrl;
			break;
		case CAN_ID_BLINK:
			sender = CAN_ZONE_N1;
			filt = &rx_blink;
			break;
		case CAN_ID_FORCE:
			sender = CAN_ZONE_N2;
			filt = &rx_force;
			break;
		case CAN_ID_HB_N1:
		case CAN_ID_HB_N2:
		case CAN_ID_HB_N3:
			sender = can_hb_zone(id);
			filt = &rx_hb[sender];
			break;
		case CAN_ID_FAULT:
			if (can_unpack_fault(d, &f) == CAN_OK) {
				sender = f.zone;
				filt = &rx_fault[sender];
			}
			break;
		default:
			break;
		}
		if (sender == NODE_ID) {
			continue; /* our own frame came back through someone's relay */
		}

		zone_on_frame(it.bus, sender, can_hb_zone(id) != 0);
		if (can_hb_zone(id) != 0) {
			struct can_hb hb;

			if (can_unpack_hb(d, &hb) == CAN_OK) {
				zone_on_hb(sender, hb.health);
			}
		}
		if (filt == NULL || !can_seq_accept(filt, can_seq(d), now)) {
			continue;
		}
		maybe_relay(&it, sender, false);

		switch (id) {
		case CAN_ID_CTRL: {
			struct can_ctrl c;
			enum can_check r = can_unpack_ctrl(d, &c);

			if (r == CAN_OK) {
				node_on_ctrl(&c);
			}
			zone_on_ctrl(r);
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
			break;
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
