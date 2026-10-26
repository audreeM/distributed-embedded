/*
 * Heartbeat monitor (HB_MON) and per-bus health for the cockpit (R4.5, R4.7).
 * main.c feeds it every received frame; health_update() turns the timestamps
 * into degraded / failed / isolated decisions.
 */
#ifndef HEALTH_H
#define HEALTH_H

#include <stdbool.h>
#include <stdint.h>

#include "can_catalog.h"
#include "canbus.h"

#define ZONE_SLOTS 4 /* index by zone number 1..3; slot 0 unused */

struct health {
	/* --- inputs, filled by health_on_frame() / health_on_heartbeat() --- */
	uint64_t last_hb_us[ZONE_SLOTS][BUS_COUNT]; /* last HB from zone z on bus b (0 = never) */
	uint64_t last_rx_us[BUS_COUNT];             /* last frame from any other node on bus b */
	struct can_hb peer_hb[ZONE_SLOTS];          /* latest heartbeat payload from each zone */
	uint64_t start_us;                          /* when monitoring started */

	/* --- outputs, set by health_update() --- */
	bool degraded[BUS_COUNT];      /* R4.7: this bus is silent while the other is alive */
	bool zone_alive[ZONE_SLOTS];   /* zone heard on at least one bus recently */
	bool zone_failed[ZONE_SLOTS];  /* R4.5: HB_MISS_LIMIT periods missed on BOTH buses */
	bool isolated;                 /* R4.7: nothing from anyone on either bus */
};

void health_init(struct health *h, uint64_t now_us);

/* Every received frame, duplicates included: per-bus liveness needs each copy. */
void health_on_frame(struct health *h, int bus, uint16_t id, uint64_t now_us);

/* A new (deduplicated, valid) heartbeat payload from a zone, for logging. */
void health_on_heartbeat(struct health *h, uint8_t zone, const struct can_hb *hb);

/* Recompute the outputs. Called every TICK_MS. */
void health_update(struct health *h, uint64_t now_us);

/* HB_N1 byte 2 (CAN_HEALTH_*_OK) from degraded[] and isolated. */
uint8_t health_bits(const struct health *h);

#endif /* HEALTH_H */
