/* TODO 3.3: steering servo */
/**
 * @file servo.c
 * @brief 3.3 steering: servo_thread runs every 20 ms and maps the wheel angle
 *        (steer -100..+100 from the Pi) to a servo pulse width.
 *
 * Signal: 50 Hz PWM on PB10 [D6] (TIM2_CH3). Pulse width sets the angle.
 * Mapping (documented, monotonic): piecewise linear through three measured
 * points, so left and right can have different travel:
 *   steer -100 -> SERVO_MIN_US, 0 -> SERVO_CENTER_US, +100 -> SERVO_MAX_US
 * Pulses never leave [MIN, MAX], so the servo never pushes against the
 * linkage end stops (no buzzing / stalling).
 *
 * Power: the servo runs from the buck converter, NOT the Nucleo. Only the
 * signal wire and a common ground go to the STM32.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/atomic.h>
#include "config.h"
#include "link.h"
#include "sys_state.h"
#include "servo.h"

static const struct pwm_dt_spec srv = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);

static atomic_t raw_us;   /* 0 = follow the wheel; otherwise calibration pulse */
static atomic_t cur_us;   /* what is on the pin right now */

/* steer -100..+100 -> pulse width, monotonic, clamped to the measured range. */
static int steer_to_us(int steer)
{
	if (SERVO_INVERT) {
		steer = -steer;
	}
	if (steer > 100) {
		steer = 100;
	} else if (steer < -100) {
		steer = -100;
	}
	if (steer >= 0) {
		return SERVO_CENTER_US + (SERVO_MAX_US - SERVO_CENTER_US) * steer / 100;
	}
	return SERVO_CENTER_US + (SERVO_CENTER_US - SERVO_MIN_US) * steer / 100;
}

K_TIMER_DEFINE(servo_timer, NULL, NULL);

static void servo_thread(void *p1, void *p2, void *p3)
{
	struct link_cmd c;

	if (!pwm_is_ready_dt(&srv)) {
		printk("servo PWM not ready\n");
		return;
	}

	k_timer_start(&servo_timer, K_MSEC(SERVO_PERIOD_MS), K_MSEC(SERVO_PERIOD_MS));

	for (;;) {
		k_timer_status_sync(&servo_timer);

		int us = atomic_get(&raw_us);

		if (us == 0) {
			link_get_cmd(&c);
			/* CHOOSE: fail-safe steering. We center the wheels in
			 * ERROR (car is braked anyway); holding the last angle
			 * is the other defensible option. */
			if (sys_state_is_error() || !c.valid) {
				us = SERVO_CENTER_US;
			} else {
				us = steer_to_us(c.steer);
			}
		}

		/* Only touch the timer when the value changes. The new width
		 * takes effect at the start of the next 20 ms PWM period. */
		if (us != atomic_get(&cur_us)) {
			pwm_set_pulse_dt(&srv, PWM_USEC(us));
			atomic_set(&cur_us, us);
		}
	}
}

K_THREAD_DEFINE(servo_tid, STACK_SZ, servo_thread, NULL, NULL, NULL, PRIO_SERVO, 0, SYS_FOREVER_MS);

void servo_set_raw_us(int us)
{
	if (us != 0) {
		if (us < SERVO_ABS_MIN_US) {
			us = SERVO_ABS_MIN_US;
		} else if (us > SERVO_ABS_MAX_US) {
			us = SERVO_ABS_MAX_US;
		}
	}
	atomic_set(&raw_us, us);
}

int servo_get_us(void)
{
	return (int)atomic_get(&cur_us);
}
