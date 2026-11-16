/**
 * @file pid.c
 * @brief Lower-priority "PID" thread, currently a pass-through: no filtering,
 *        no ramp, no feed-forward, no PID math. Every CONTROL_PERIOD_MS it takes
 *        the commanded value and hands it unchanged to the motor thread with
 *        motor_set_duty(). The motor thread still brakes on fault / no command /
 *        brake, so that safety path does not depend on this thread.
 */
#include <zephyr/kernel.h>

#include "config.h"
#include "motor.h"
#include "pid.h"
#include "zone.h"

static void pid_thread(void *p1, void *p2, void *p3)
{
	struct motor_cmd c;

	/* Absolute deadline so the period doesn't drift with the loop's run time. */
	const int64_t period_ticks = k_ms_to_ticks_ceil64(HB_PERIOD_MS);
	int64_t next = k_uptime_ticks() + period_ticks;

	for (;;) {
		k_sleep(K_TIMEOUT_ABS_TICKS(next));
		next += period_ticks;
		zone_wdt_checkin(WDT_PID);

		motor_get_cmd(&c);
		motor_set_duty(c.throttle);   /* written as given, no calculation */
	}
}

K_THREAD_DEFINE(pid_tid, STACK_SZ, pid_thread, NULL, NULL, NULL, PRIO_PID, 0, SYS_FOREVER_MS);

int pid_init(void)
{
	k_thread_start(pid_tid);
	return 0;
}