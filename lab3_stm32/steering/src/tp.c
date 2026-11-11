#include <zephyr/drivers/gpio.h>
#include "tp.h"

#define USER DT_PATH(zephyr_user)

static const struct gpio_dt_spec pins[TP_COUNT] = {
	[TP_CAN_RX_A] = GPIO_DT_SPEC_GET(USER, canrxa_gpios),
	[TP_CAN_RX_B] = GPIO_DT_SPEC_GET(USER, canrxb_gpios),
	[TP_CAN_TX_A] = GPIO_DT_SPEC_GET(USER, cantxa_gpios),
	[TP_CAN_TX_B] = GPIO_DT_SPEC_GET(USER, cantxb_gpios),
	[TP_DEG_A] = GPIO_DT_SPEC_GET(USER, dega_gpios),
	[TP_DEG_B] = GPIO_DT_SPEC_GET(USER, degb_gpios),
	[TP_RELAY_TX] = GPIO_DT_SPEC_GET(USER, relaytx_gpios),
	[TP_ERR] = GPIO_DT_SPEC_GET(USER, err_gpios),
};

int tp_init(void)
{
	for (int i = 0; i < TP_COUNT; i++) {
		if (!gpio_is_ready_dt(&pins[i]) ||
		    gpio_pin_configure_dt(&pins[i], GPIO_OUTPUT_INACTIVE) < 0) {
			return -ENODEV;
		}
	}
	gpio_pin_set_dt(&pins[TP_ERR], 1);
	return 0;
}

void tp_toggle(enum tp tp)
{
	gpio_pin_toggle_dt(&pins[tp]);
}

void tp_set(enum tp tp, bool on)
{
	gpio_pin_set_dt(&pins[tp], on);
}
