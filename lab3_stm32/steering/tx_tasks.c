/*
 * STEER (task table: on CTRL, 50 ms end to end, 1 ms local).
 *
 * Servo signal and mapping are lab2_stm32/src/servo.c: 50 Hz PWM on PB10
 * (TIM2_CH3), steer -100..+100 -> pulse width, piecewise linear through the
 * measured MIN / CENTER / MAX, never outside them. Fail-safe: center the
 * wheels whenever the zone is not NORMAL (same choice as Lab 2).
 *
 * Woken by every new CTRL and by state changes; also wakes every 20 ms so a
 * missed wake-up can never leave the servo at a stale angle.
 */
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "config.h"
#include "steer.h"
#include "zone.h"

static const struct pwm_dt_spec srv = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static struct k_spinlock lock;
static int16_t steer_cmd;
static bool have_cmd;
static K_SEM_DEFINE(steer_sem, 0, 1);

/* From lab2_stm32/src/servo.c (steer_to_us()), unchanged. */
static int steer_to_us(int steer)
{
	if (SERVO_INVERT) {
		steer = -steer;
	}
	if (steer > 50) {
		steer = 100;
	} else if (steer < -50) {
		steer = -100;
	}
	if (steer >= 0) {
		return SERVO_CENTER_US + (SERVO_MAX_US - SERVO_CENTER_US) * steer / 100;
	}
	return SERVO_CENTER_US + (SERVO_CENTER_US - SERVO_MIN_US) * steer / 100;
}

void steer_on_ctrl(const struct can_ctrl *c)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	steer_cmd = c->steer;
	have_cmd = true;
	k_spin_unlock(&lock, key);
	k_sem_give(&steer_sem);
}

void steer_kick(void)
{
	k_sem_give(&steer_sem);
}

static void steer_thread(void *p1, void *p2, void *p3)
{
	int cur_us = 0;

	for (;;) {
		(void)k_sem_take(&steer_sem, K_MSEC(HB_PERIOD_MS));
		zone_wdt_checkin(WDT_STEER);

		k_spinlock_key_t key = k_spin_lock(&lock);
		int16_t s = steer_cmd;
		bool ok = have_cmd;

		k_spin_unlock(&lock, key);

		int us = (ok && zone_state() == ZONE_NORMAL) ? steer_to_us(s) : SERVO_CENTER_US;

		/* Only touch the timer when the value changes; the new width takes
		 * effect at the start of the next 20 ms PWM period. */
		if (us != cur_us && pwm_set_pulse_dt(&srv, PWM_USEC(us)) == 0) {
			cur_us = us;
		}
	}
}

K_THREAD_DEFINE(steer_tid, STACK_SZ, steer_thread, NULL, NULL, NULL, PRIO_STEER, 0,
		SYS_FOREVER_MS);

int steer_init(void)
{
	if (!pwm_is_ready_dt(&srv)) {
		printk("servo PWM not ready\n");
		return -ENODEV;
	}
	k_thread_start(steer_tid);
	return 0;
}
