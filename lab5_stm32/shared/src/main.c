/*
 * Lab 5 STM32 node (N2 steering or N3 drivetrain, picked by NODE_ID).
 *
 * main() starts everything, then every 100 ms feeds the watchdog (only if all
 * periodic threads checked in, like lab 3) and prints:
 *   - one PPS line per second (part 6.4): global time, local time, drift
 *     ratio and the correction applied at the last REF
 *   - a status line every second: zone state, CAN counters, raft, sync
 */
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#include "canbus.h"
#include "config.h"
#include "edges.h"
#include "gtime.h"
#include "node.h"
#include "raft_task.h"
#include "relay.h"
#include "tp.h"
#include "zone.h"

int dispatch_init(void);
int hb_tx_init(void);

static const struct device *const wdt = DEVICE_DT_GET(DT_NODELABEL(iwdg));

static int wdt_start(void)
{
	struct wdt_timeout_cfg cfg = {
		.window = {.min = 0, .max = WDT_TIMEOUT_MS},
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

/* part 6.4: one line per PPS edge */
static void print_pps(const struct pps_record *r)
{
	char drift[24];

	gtime_fmt_drift(drift, sizeof(drift), r->drift_ppb);
	printk("PPS global %llu.%06llu s | local %llu.%06llu s | drift %s | last correction %+lld us | %s\n",
	       r->global_us / 1000000u, r->global_us % 1000000u, r->local_us / 1000000u,
	       r->local_us % 1000000u, drift, (long long)r->last_error_us,
	       r->leader ? "LEADER (time master)" : r->synced ? "synced" : "NOT synced");
}

static void print_status(void)
{
	struct canbus_stats st;
	struct gtime_stats gs;
	char drift[24];

	canbus_get_stats(&st);
	gtime_get_stats(&gs);
	gtime_fmt_drift(drift, sizeof(drift), gs.drift_ppb);
	printk("%-10s | A tx %u fail %u rx %u | B tx %u fail %u rx %u | health 0x%x | relayed %u | "
	       "rxq drop %u%s\n",
	       state_str(zone_state()), st.tx_ok[BUS_A], st.tx_fail[BUS_A], st.rx[BUS_A],
	       st.tx_ok[BUS_B], st.tx_fail[BUS_B], st.rx[BUS_B], zone_health_bits(), relay_count(),
	       st.rx_drop, raft_task_failure_mode() ? " | FAILURE MODE" : "");
	raft_task_print_status();
	/* part 6.2: the drift correction currently applied */
	printk("sync: %s drift %s, last correction %+lld us, %u samples (%u rejected)\n",
	       gs.leader ? "time master," : gs.synced ? "synced," : "NOT synced,", drift,
	       (long long)gs.last_error_us, gs.samples, gs.rejected);
}

int main(void)
{
	struct pps_record pps;
	int64_t next_print = 0;
	int wdt_ch;

	printk("\nLab 5 N%d %s zone\n", NODE_ID, node_name);

	if (tp_init() < 0) {
		printk("some test point GPIOs not ready\n");
	}
	/* outputs first, in their safe state (ERROR: braked / centered, hazards) */
	node_outputs_init();
	edges_init();
	zone_init();
	relay_init();
	raft_task_init();
	dispatch_init();
	if (canbus_init() < 0) {
		printk("no CAN module came up\n");
	}
	hb_tx_init();
	node_tx_init();

	wdt_ch = wdt_start();
	if (wdt_ch < 0) {
		printk("watchdog not started (%d)\n", wdt_ch);
	}

	for (;;) {
		k_msleep(WDT_FEED_MS);

		/* feed only if every periodic thread ran since the last check */
		if (wdt_ch >= 0 && (zone_wdt_take() & WDT_ALL) == WDT_ALL) {
			wdt_feed(wdt, wdt_ch);
		}
		while (k_msgq_get(&pps_q, &pps, K_NO_WAIT) == 0) {
			print_pps(&pps);
		}
		if (k_uptime_get() >= next_print) {
			next_print = k_uptime_get() + STATUS_PRINT_MS;
			print_status();
		}
	}
	return 0;
}
