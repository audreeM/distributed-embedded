#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "blink.h"
#include "config.h"
#include "zone.h"

enum { FL, FR, N_LEDS };

static const struct gpio_dt_spec leds[N_LEDS] = {
	GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), blink_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), blink_gpios, 1),
};

K_MSGQ_DEFINE(blink_q, sizeof(struct can_blink), 4, 1);

void blink_on_msg(const struct can_blink *b)
{
	(void)k_msgq_put(&blink_q, b, K_NO_WAIT);
}

#define MODE_RECHECK 0xFF

void blink_kick(void)
{
	const struct can_blink recheck = { .mode = MODE_RECHECK };

	(void)k_msgq_put(&blink_q, &recheck, K_NO_WAIT);
}

static void apply(const struct can_blink *b)
{
	bool left = (b->mode == CAN_BLINK_LEFT || b->mode == CAN_BLINK_HAZARD) && b->phase;
	bool right = (b->mode == CAN_BLINK_RIGHT || b->mode == CAN_BLINK_HAZARD) && b->phase;

	gpio_pin_set_dt(&leds[FL], left);
	gpio_pin_set_dt(&leds[FR], right);
}

static void blink_thread(void *p1, void *p2, void *p3)
{
	struct can_blink b;
	struct can_blink local = { .mode = CAN_BLINK_HAZARD, .phase = 1 };

	/* R3: power up in ERROR with hazards until the cockpit takes over. */
	apply(&local);

	for (;;) {
		int got = k_msgq_get(&blink_q, &b, K_MSEC(HAZARD_HALF_MS));
		bool own_hazards = zone_state() != ZONE_NORMAL && !zone_blink_master_alive();

		if (got == 0 && b.mode != MODE_RECHECK) {
			apply(&b);
		} else if (own_hazards) {
			local.phase = got == 0 ? 1 : local.phase ^ 1;
			apply(&local);
		}
	}
}

K_THREAD_DEFINE(blink_tid, STACK_SZ, blink_thread, NULL, NULL, NULL, PRIO_BLINK_F, 0,
		SYS_FOREVER_MS);

int blink_init(void)
{
	for (int i = 0; i < N_LEDS; i++) {
		if (!gpio_is_ready_dt(&leds[i]) ||
		    gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE) < 0) {
			printk("blinker GPIO not ready\n");
			return -ENODEV;
		}
	}
	k_thread_start(blink_tid);
	return 0;
}
