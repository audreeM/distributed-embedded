#include <string.h>

#include "config.h"
#include "health.h"

void health_init(struct health *h, uint64_t now_us)
{
	memset(h, 0, sizeof(*h));
	h->start_us = now_us;
}

void health_on_frame(struct health *h, int bus, uint16_t id, uint64_t now_us)
{
	uint8_t zone = can_hb_zone(id);

	h->last_rx_us[bus] = now_us;
	if (zone != 0) {
		h->last_hb_us[zone][bus] = now_us;
	}
}

void health_on_heartbeat(struct health *h, uint8_t zone, const struct can_hb *hb)
{
	h->peer_hb[zone] = *hb;
}

void health_update(struct health *h, uint64_t now_us)
{
	/*
	 * TODO(cockpit): fill the four outputs from the timestamps above.
	 * A timestamp of 0 means "never seen"; measure silence from start_us then.
	 *
	 * 1. zone_alive[z], for z = CAN_ZONE_N2 and CAN_ZONE_N3:
	 *      heard an HB_Nz on bus A or bus B within the last
	 *      HB_MISS_LIMIT * HB_PERIOD_MS.
	 *
	 * 2. zone_failed[z]  (R4.5):
	 *      HB_MISS_LIMIT heartbeats missed on BOTH buses, i.e. neither
	 *      last_hb_us[z][BUS_A] nor last_hb_us[z][BUS_B] is newer than
	 *      HB_MISS_LIMIT * HB_PERIOD_MS. Missing on one bus only is NOT a
	 *      zone failure; that is a degraded bus (item 3).
	 *      Decide what "the 3rd heartbeat is missed" means with the 20 ms
	 *      +10 % tolerance (66 ms?) and write it into the analysis.
	 *
	 * 3. degraded[b]  (R4.7):
	 *      no frame from ANY other node on bus b for DEGRADED_MS while
	 *      frames are still arriving on the other bus. Must clear within
	 *      100 ms of traffic coming back (it will, since last_rx_us moves).
	 *
	 * 4. isolated  (R4.7):
	 *      nothing from anyone on either bus for HB_MISS_LIMIT periods.
	 *      Then the cockpit is cut off, not degraded: clear both
	 *      degraded[] flags so the LEDs don't claim a single-bus fault.
	 */
	(void)now_us;
	(void)h;
}

uint8_t health_bits(const struct health *h)
{
	return (h->degraded[BUS_A] ? CAN_HEALTH_A_DEGRADED : 0) |
	       (h->degraded[BUS_B] ? CAN_HEALTH_B_DEGRADED : 0);
}
