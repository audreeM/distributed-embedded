#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "config.h"

K_THREAD_DEFINE(rx_tid,      STACK_SZ, rx_thread,      NULL, NULL, NULL, PRIO_RX,      0, SYS_FOREVER_MS);
K_THREAD_DEFINE(monitor_tid, STACK_SZ, monitor_thread, NULL, NULL, NULL, PRIO_MONITOR, 0, SYS_FOREVER_MS);
K_THREAD_DEFINE(ctrl_tid,    STACK_SZ, ctrl_thread,    NULL, NULL, NULL, PRIO_CTRL,    0, SYS_FOREVER_MS);
K_THREAD_DEFINE(hb_tid,      STACK_SZ, hb_thread,      NULL, NULL, NULL, PRIO_HB,      0, SYS_FOREVER_MS);
K_THREAD_DEFINE(force_tid,   STACK_SZ, force_thread,   NULL, NULL, NULL, PRIO_FORCE,   0, SYS_FOREVER_MS);

int main(void)
{
	for (int i = 0; i < TP_COUNT; i++) {
		gpio_pin_configure_dt(&tp[i], GPIO_OUTPUT_INACTIVE);
	}

	k_thread_start(rx_tid);
	k_thread_start(monitor_tid);
	k_thread_start(ctrl_tid);
	k_thread_start(hb_tid);
	k_thread_start(force_tid);

	return 0;
}