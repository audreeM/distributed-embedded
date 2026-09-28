/**
 * @file status.c
 * @brief status_thread runs every 20 ms: sends the status frame (state + current) back to the Pi
 */
#include "config.h"

#define STATUS_PERIOD_MS 20

K_TIMER_DEFINE(status_timer, NULL, NULL);

static void status_thread(void *p1, void *p2, void *p3)
{
	// periodic timer instead of k_msleep so the heartbeat stays within 20 ms +-10%
	k_timer_start(&status_timer, K_MSEC(STATUS_PERIOD_MS), K_MSEC(STATUS_PERIOD_MS));

	for (;;) {
		k_timer_status_sync(&status_timer);
		// TODO: build status frame from state + latest current readings, send to Pi
	}
}

K_THREAD_DEFINE(status_tid, STACK_SZ, status_thread, NULL, NULL, NULL, PRIO_STATUS, 0, SYS_FOREVER_MS);
