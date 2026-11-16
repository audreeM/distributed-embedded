#include <zephyr/drivers/gpio.h>

#include "tp.h"

#define USER DT_PATH(zephyr_user)
#define PIN(prop) GPIO_DT_SPEC_GET_OR(USER, prop, {0})

static const struct gpio_dt_spec pins[TP_COUNT] = {
	[TP_CAN_RX_A] = PIN(canrxa_gpios),
	[TP_CAN_RX_B] = PIN(canrxb_gpios),
	[TP_CAN_TX_A] = PIN(cantxa_gpios),
	[TP_CAN_TX_B] = PIN(cantxb_gpios),
	[TP_DEG_A] = PIN(dega_gpios),
	[TP_DEG_B] = PIN(degb_gpios),
	[TP_RELAY_TX] = PIN(relaytx_gpios),
	[TP_ERR] = PIN(err_gpios),
	[TP_LEADER] = PIN(leader_gpios),
	[TP_SYNC] = PIN(sync_gpios),
	[TP_REF_RX] = PIN(refrx_gpios),
	[TP_PPS] = PIN(pps_gpios),
};

int tp_init(void)
{
	int err = 0;

	for (int i = 0; i < TP_COUNT; i++) {
		if (pins[i].port == NULL) {
			continue; /* not wired on this board */
		}
		if (!gpio_is_ready_dt(&pins[i]) || gpio_pin_configure_dt(&pins[i], GPIO_OUTPUT_INACTIVE) < 0) {
			err = -ENODEV;
		}
	}
	tp_set(TP_ERR, true);
	return err;
}

void tp_toggle(enum tp tp)
{
	if (pins[tp].port != NULL) {
		gpio_pin_toggle_dt(&pins[tp]);
	}
}

void tp_set(enum tp tp, bool on)
{
	if (pins[tp].port != NULL) {
		gpio_pin_set_dt(&pins[tp], on);
	}
}
