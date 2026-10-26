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
	if (zone != 0)
	{
		h->last_hb_us[zone][bus] = now_us;
	}
}

void health_on_heartbeat(struct health *h, uint8_t zone, const struct can_hb *hb)
{
	h->peer_hb[zone] = *hb;
}

/* How long since last_us, in us. A timestamp of 0 means "never seen";
 * measure silence from start_us then. */
static uint64_t silent_us(const struct health *h, uint64_t last_us, uint64_t now_us)
{
	uint64_t since = last_us ? last_us : h->start_us;
	return now_us > since ? now_us - since : 0;
}

void health_update(struct health *h, uint64_t now_us)
{
	const uint64_t hb_limit_us = (uint64_t)HB_MISS_LIMIT * HB_PERIOD_MS * 1000;
	const uint64_t degraded_us = (uint64_t)DEGRADED_MS * 1000;

	/*
	 * 1. zone_alive[z]: heard an HB_Nz on bus A or bus B within the last
	 *    HB_MISS_LIMIT * HB_PERIOD_MS.
	 * 2. zone_failed[z] (R4.5): HB_MISS_LIMIT heartbeats missed on BOTH
	 *    buses. Missing on one bus only is NOT a zone failure; that is a
	 *    degraded bus (item 3).
	 *    "The 3rd heartbeat is missed" is taken as 60 ms of silence on both
	 *    buses (3 x 20 ms nominal), not 66 ms (3 x 22 ms with the +10 %
	 *    tolerance): a peer running slow but in spec still sends within
	 *    22 ms, so it never gets near 60 ms, and failing fast is safer.
	 */
	for (uint8_t z = CAN_ZONE_N2; z <= CAN_ZONE_N3; z++)
	{
		bool fresh_a = silent_us(h, h->last_hb_us[z][BUS_A], now_us) < hb_limit_us;
		bool fresh_b = silent_us(h, h->last_hb_us[z][BUS_B], now_us) < hb_limit_us;

		h->zone_alive[z] = fresh_a || fresh_b;
		h->zone_failed[z] = !fresh_a && !fresh_b;
	}

	/* 3. degraded[b] (R4.7): no frame from ANY other node on bus b for
	 *    DEGRADED_MS while frames are still arriving on the other bus.
	 *    Clears as soon as traffic comes back, since last_rx_us moves.
	 */
	bool quiet_a = silent_us(h, h->last_rx_us[BUS_A], now_us) >= degraded_us;
	bool quiet_b = silent_us(h, h->last_rx_us[BUS_B], now_us) >= degraded_us;

	h->degraded[BUS_A] = quiet_a && !quiet_b;
	h->degraded[BUS_B] = quiet_b && !quiet_a;

	/* 4. isolated (R4.7): nothing from anyone on either bus for
	 *    HB_MISS_LIMIT periods. Then the cockpit is cut off, not degraded:
	 *    clear both degraded[] flags so the LEDs don't claim a single-bus fault.
	 */
	h->isolated = silent_us(h, h->last_rx_us[BUS_A], now_us) >= hb_limit_us &&
				  silent_us(h, h->last_rx_us[BUS_B], now_us) >= hb_limit_us;
	if (h->isolated)
	{
		h->degraded[BUS_A] = false;
		h->degraded[BUS_B] = false;
	}
}

uint8_t health_bits(const struct health *h)
{
	/* A bus is ok unless it is degraded or we're cut off from both. */
	if (h->isolated)
	{
		return 0;
	}
	return (h->degraded[BUS_A] ? 0 : CAN_HEALTH_A_OK) |
		   (h->degraded[BUS_B] ? 0 : CAN_HEALTH_B_OK);
}
