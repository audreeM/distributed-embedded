#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#include "blink.h"
#include "canbus.h"
#include "config.h"
#include "relay.h"
#include "steer.h"
#include "tp.h"
#include "zone.h"

int dispatch_init(void);
int tx_tasks_init(void);

static const struct device *const wdt = DEVICE_DT_GET(DT_NODELABEL(iwdg));

static int wdt_start(void)
{
	struct wdt_timeout_cfg cfg = {
		.window = { .min = 0, .max = WDT_TIMEOUT_MS },
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};
	int ch;

	if (!device_is_ready(wdt)) {
		return -ENODEV;
	}
	ch = wdt_install_timeout(wdt, &cfg);
	if (ch < 0 || wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) < 0) {
		return -EIO;
	}
	return ch;
}

static const char *state_str(enum zone_state s)
{
	return s == ZONE_NORMAL ? "NORMAL" : s == ZONE_ERROR ? "ERROR" : "LOCAL_FAIL";
}

int main(void)
{
	struct canbus_stats st;
	int64_t next_print = 0;
	int wdt_ch;

	printk("\nLab 3 N2 steering zone\n");

	if (tp_init() < 0) {
		printk("test point GPIO not ready\n");
	}
	/* Outputs first, in their safe state (ERROR: centered servo, hazards). */
	steer_init();
	blink_init();
	zone_init();
	relay_init();
	dispatch_init();
	if (canbus_init() < 0) {
		printk("no CAN module came up\n");
	}
	tx_tasks_init();

	wdt_ch = wdt_start();
	if (wdt_ch < 0) {
		printk("watchdog not started (%d)\n", wdt_ch);
	}

	for (;;) {
		k_msleep(WDT_FEED_MS);

		/* Feed only if every periodic thread ran since the last check, so a
		 * stuck thread resets the board instead of hiding. */
		if (wdt_ch >= 0 && (zone_wdt_take() & WDT_ALL) == WDT_ALL) {
			wdt_feed(wdt, wdt_ch);
		}

		if (k_uptime_get() >= next_print) {
			next_print = k_uptime_get() + STATUS_PRINT_MS;
			canbus_get_stats(&st);
			printk("%-10s | A tx %u fail %u rx %u st %d | B tx %u fail %u rx %u st %d | "
			       "health 0x%x | relayed %u | rxq drop %u\n",
			       state_str(zone_state()), st.tx_ok[BUS_A], st.tx_fail[BUS_A],
			       st.rx[BUS_A], st.state[BUS_A], st.tx_ok[BUS_B], st.tx_fail[BUS_B],
			       st.rx[BUS_B], st.state[BUS_B], zone_health_bits(), relay_count(),
			       st.rx_drop);
		}
	}
	return 0;
}
