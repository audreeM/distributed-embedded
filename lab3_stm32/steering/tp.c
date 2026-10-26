/*
 * HB_TX (20 ms) and FORCE_TX (50 ms). Both go through canbus_send_both(),
 * one sequence counter per ID, and both stop while the zone is in
 * LOCAL_FAIL (a failed zone goes silent, R2.5 / R4.5).
 */
#include <zephyr/kernel.h>

#include "canbus.h"
#include "config.h"
#include "zone.h"

K_TIMER_DEFINE(hb_tx_timer, NULL, NULL);
K_TIMER_DEFINE(force_tx_timer, NULL, NULL);

static void hb_tx_thread(void *p1, void *p2, void *p3)
{
	uint8_t seq = 0;

	/* Fixed 20 ms grid from a k_timer, so the period does not drift (R4.5). */
	k_timer_start(&hb_tx_timer, K_MSEC(HB_PERIOD_MS), K_MSEC(HB_PERIOD_MS));
	for (;;) {
		k_timer_status_sync(&hb_tx_timer);
		if (zone_state() != ZONE_LOCAL_FAIL) {
			struct can_hb hb = { .state = zone_wire_state(), .health = zone_health_bits() };
			uint8_t d[CAN_DLC];

			can_pack_hb(d, seq, &hb);
			canbus_send_both(CAN_ID_HB_N2, d, &seq);
		}
		zone_wdt_checkin(WDT_HB_TX);
	}
}

static void force_tx_thread(void *p1, void *p2, void *p3)
{
	uint8_t seq = 0;
	int8_t force = 0; /* Lab 3: a dummy counter, kept inside -100..+100 */

	k_timer_start(&force_tx_timer, K_MSEC(FORCE_PERIOD_MS), K_MSEC(FORCE_PERIOD_MS));
	for (;;) {
		k_timer_status_sync(&force_tx_timer);
		if (zone_state() != ZONE_LOCAL_FAIL) {
			struct can_force f = { .force = force };
			uint8_t d[CAN_DLC];

			can_pack_force(d, seq, &f);
			canbus_send_both(CAN_ID_FORCE, d, &seq);
			force = force >= 100 ? -100 : force + 1;
		}
		zone_wdt_checkin(WDT_FORCE_TX);
	}
}

K_THREAD_DEFINE(hb_tx_tid, STACK_SZ, hb_tx_thread, NULL, NULL, NULL, PRIO_HB_TX, 0,
		SYS_FOREVER_MS);
K_THREAD_DEFINE(force_tx_tid, STACK_SZ, force_tx_thread, NULL, NULL, NULL, PRIO_FORCE_TX, 0,
		SYS_FOREVER_MS);

int tx_tasks_init(void)
{
	k_thread_start(hb_tx_tid);
	k_thread_start(force_tx_tid);
	return 0;
}
