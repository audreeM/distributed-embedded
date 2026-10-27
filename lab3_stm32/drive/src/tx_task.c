#include <zephyr/kernel.h>

#include "canbus.h"
#include "config.h"
#include "zone.h"

K_TIMER_DEFINE(hb_tx_timer, NULL, NULL);

static void hb_tx_thread(void *p1, void *p2, void *p3)
{
	uint8_t seq = 0;

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

K_THREAD_DEFINE(hb_tx_tid, STACK_SZ, hb_tx_thread, NULL, NULL, NULL, PRIO_HB_TX, 0,
		SYS_FOREVER_MS);

int tx_task_init(void)
{
	k_thread_start(hb_tx_tid);
	return 0;
}