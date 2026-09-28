/**
 * @file sense.c
 * @brief sense_thread runs every 10 ms: reads the three current sensors
 */
#include "config.h"

#define SENSE_PERIOD_MS 10

K_TIMER_DEFINE(sense_timer, NULL, NULL);

static void sense_thread(void *p1, void *p2, void *p3)
{
	k_timer_start(&sense_timer, K_MSEC(SENSE_PERIOD_MS), K_MSEC(SENSE_PERIOD_MS));

	for (;;) {
		k_timer_status_sync(&sense_timer);
		// TODO: sample the three ADC channels and store the latest readings
	}
}

K_THREAD_DEFINE(sense_tid, STACK_SZ, sense_thread, NULL, NULL, NULL, PRIO_SENSE, 0, SYS_FOREVER_MS);
