/**
 * @file motor.c
 * @brief motor_thread runs every 10 ms: reads the encoders and updates the motors
 */
#include "config.h"

#define MOTOR_PERIOD_MS 10

K_TIMER_DEFINE(motor_timer, NULL, NULL);

static void motor_thread(void *p1, void *p2, void *p3)
{
	// periodic timer instead of k_msleep so the period doesn't drift by the loop's run time
	k_timer_start(&motor_timer, K_MSEC(MOTOR_PERIOD_MS), K_MSEC(MOTOR_PERIOD_MS));

	for (;;) {
		k_timer_status_sync(&motor_timer);
		// TODO: read encoders, run PI loop, write PWM
	}
}

K_THREAD_DEFINE(motor_tid, STACK_SZ, motor_thread, NULL, NULL, NULL, PRIO_MOTOR, 0, SYS_FOREVER_MS);
