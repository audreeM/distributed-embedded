/*
 * The cockpit zone's own state machine (R3) and its test button (R2.5, S5).
 * This must match the system state chart in the design doc.
 */
#ifndef ZONE_H
#define ZONE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_catalog.h"

enum zone_state {
	ZONE_ERROR,      /* system-wide error state; also the power-up state */
	ZONE_NORMAL,
	ZONE_LOCAL_FAIL, /* this zone failed itself: test button or UDP lost; silent on CAN */
};

struct zone {
	enum zone_state state;
	uint8_t fault_to_send;    /* reason for one FAULT frame (CAN_FAULT_*), 0 = none */
	bool self_test;           /* failed by the test button; only a double press clears it */

	/* TODO(cockpit): add what the rest of the state logic needs, e.g. which
	 * zone sent a FAULT and when, so a stale FAULT can be cleared.
	 */
};

struct zone_inputs {
	bool udp_fresh;    /* a UDP update arrived within UDP_STALE_MS */
	bool test_btn;     /* wheel test button pressed right now (raw, not debounced) */
	bool peers_alive;  /* heartbeats from N2 and N3, each on at least one bus */
	bool peer_failed;  /* N2 or N3 missed HB_MISS_LIMIT heartbeats on both buses */
	bool isolated;     /* heard nobody on either bus */
};

void zone_init(struct zone *z);

/* Called every TICK_MS with fresh inputs. */
void zone_update(struct zone *z, const struct zone_inputs *in, uint64_t now_us);

/* A valid, deduplicated FAULT frame from another zone. */
void zone_on_fault(struct zone *z, const struct can_fault *f, uint64_t now_us);

/* LOCAL_FAIL means the zone stops transmitting on both buses (R2.5, R4.5). */
static inline bool zone_transmitting(const struct zone *z)
{
	return z->state != ZONE_LOCAL_FAIL;
}

/* HB_N1 byte 1. */
static inline uint8_t zone_wire_state(const struct zone *z)
{
	return z->state == ZONE_NORMAL ? CAN_STATE_NORMAL : CAN_STATE_ERROR;
}

#endif /* ZONE_H */
