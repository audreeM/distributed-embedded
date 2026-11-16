/* N2 steering: the node-specific hooks (see shared/include/node.h), plus FORCE_TX. */
#include <zephyr/kernel.h>

#include "canbus.h"
#include "config.h"
#include "node.h"
#include "steer.h"
#include "zone.h"

const char *const node_name = "steering";

int node_outputs_init(void)
{
	return steer_init(); /* servo thread, starts centered */
}

void node_on_ctrl(const struct can_ctrl *c)
{
	steer_on_ctrl(c);
}

void node_kick_outputs(void)
{
	steer_kick();
}

/* FORCE_TX every 50 ms, from lab 3 (a dummy counter in -100..+100). */
K_TIMER_DEFINE(force_tx_timer, NULL, NULL);

static void force_tx_thread(void *p1, void *p2, void *p3)
{
	uint8_t seq = 0;
	int8_t force = 0;

	k_timer_start(&force_tx_timer, K_MSEC(FORCE_PERIOD_MS), K_MSEC(FORCE_PERIOD_MS));
	for (;;) {
		k_timer_status_sync(&force_tx_timer);
		if (zone_state() != ZONE_LOCAL_FAIL) {
			struct can_force f = {.force = force};
			uint8_t d[CAN_DLC];

			can_pack_force(d, seq, &f);
			canbus_send_both(CAN_ID_FORCE, d, &seq);
			force = force >= 100 ? -100 : force + 1;
		}
		zone_wdt_checkin(WDT_FORCE_TX);
	}
}

K_THREAD_DEFINE(force_tx_tid, STACK_SZ, force_tx_thread, NULL, NULL, NULL, PRIO_FORCE_TX, 0,
		SYS_FOREVER_MS);

int node_tx_init(void)
{
	k_thread_start(force_tx_tid);
	return 0;
}
