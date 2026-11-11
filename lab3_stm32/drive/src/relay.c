#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "config.h"
#include "relay.h"
#include "tp.h"

struct relay_item {
	struct can_frame frame;
	uint8_t to_bus;
};

K_MSGQ_DEFINE(relay_q, sizeof(struct relay_item), 16, 4);
static atomic_t relayed;

void relay_queue(const struct can_frame *f, int to_bus)
{
	struct relay_item it = { .frame = *f, .to_bus = (uint8_t)to_bus };

	(void)k_msgq_put(&relay_q, &it, K_NO_WAIT);
}

uint32_t relay_count(void)
{
	return atomic_get(&relayed);
}

static void relay_thread(void *p1, void *p2, void *p3)
{
	struct relay_item it;

	for (;;) {
		k_msgq_get(&relay_q, &it, K_FOREVER);
		// tp_toggle(TP_RELAY_TX);
		if (canbus_send_on(it.to_bus, it.frame.id, it.frame.data) == 0) {
			atomic_inc(&relayed);
		}
	}
}

K_THREAD_DEFINE(relay_tid, STACK_SZ, relay_thread, NULL, NULL, NULL, PRIO_RELAY, 0,
		SYS_FOREVER_MS);

int relay_init(void)
{
	k_thread_start(relay_tid);
	return 0;
}