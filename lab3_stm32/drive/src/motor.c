/**
 * @file motor.c
 * @brief High-priority motor thread. It is the ONLY code that touches the
 *        L298N pins / PWM. It wakes on:
 *          - a new CTRL command   (motor_on_ctrl)  -> brake at once (R2.2)
 *          - a zone state change  (motor_kick)     -> brake at once
 *          - a new duty from PID  (motor_set_duty) -> apply it
 *        The PID loop lives in pid.c at a lower priority.
 *
 * L298N truth table, per motor:
 *   EN low              -> coast (motor free-wheels)
 *   EN high, IN1 != IN2 -> drive (which one is high sets direction)
 *   EN high, IN1 == IN2 -> dynamic brake (motor terminals shorted together)
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>

#include "config.h"
#include "motor.h"
#include "zone.h"

#define USER DT_PATH(zephyr_user)

static const struct pwm_dt_spec en[2] = {
	PWM_DT_SPEC_GET_BY_IDX(USER, 0),
	PWM_DT_SPEC_GET_BY_IDX(USER, 1),
};
static const struct gpio_dt_spec in[4] = {
	GPIO_DT_SPEC_GET_BY_IDX(USER, motor_dir_gpios, 0),   /* IN1 */
	GPIO_DT_SPEC_GET_BY_IDX(USER, motor_dir_gpios, 1),   /* IN2 */
	GPIO_DT_SPEC_GET_BY_IDX(USER, motor_dir_gpios, 2),   /* IN3 */
	GPIO_DT_SPEC_GET_BY_IDX(USER, motor_dir_gpios, 3),   /* IN4 */
};
static const bool invert[2] = { MOTOR_L_INVERT, MOTOR_R_INVERT };

/* PWM_SET test point: toggled right after a new command is applied, so the
 * scope shows CMD_RX -> PWM_SET = software response time for R2.1. */
static const struct gpio_dt_spec tp_pwm_set = GPIO_DT_SPEC_GET(USER, pwmset_gpios);

static struct k_spinlock cmd_lock;
static struct motor_cmd cmd;          /* guarded by cmd_lock */
static atomic_t duty_req;             /* written by PID, read by motor thread */
static K_SEM_DEFINE(motor_sem, 0, 1);

/* ---------- shared-state API ---------- */

void motor_on_ctrl(const struct can_ctrl *c)
{
	k_spinlock_key_t key = k_spin_lock(&cmd_lock);

	cmd.valid = true;
	cmd.throttle = c->throttle;
	cmd.brake = c->brake;
	cmd.seq++;
	k_spin_unlock(&cmd_lock, key);
	k_sem_give(&motor_sem);
}

void motor_kick(void)
{
	k_sem_give(&motor_sem);
}

void motor_set_duty(uint16_t duty)
{
	atomic_set(&duty_req, duty);
	k_sem_give(&motor_sem);
}

void motor_get_cmd(struct motor_cmd *out)
{
	k_spinlock_key_t key = k_spin_lock(&cmd_lock);

	*out = cmd;
	k_spin_unlock(&cmd_lock, key);
}

/* ---------- motor thread (higher priority than PID) ---------- */

static void motor_thread(void *p1, void *p2, void *p3)
{
	uint8_t pwm_set_seq = 0;

	for (;;) {
		/* Timeout keeps the watchdog check-in alive even if nothing arrives. */
		k_sem_take(&motor_sem, K_MSEC(HB_PERIOD_MS));
		zone_wdt_checkin(WDT_MOTOR);

		struct motor_cmd c;

		motor_get_cmd(&c);

		/* Safe state on fault or no command yet; brake beats throttle. */
		if (zone_state() != ZONE_NORMAL || !c.valid || c.brake > 0) {
			motor_brake();
		} else {
			uint16_t duty = (uint16_t)atomic_get(&duty_req);

			motor_forward(duty);   /* duty 0 -> coast */
		}

		if (c.seq != pwm_set_seq) {
			gpio_pin_toggle_dt(&tp_pwm_set);
			pwm_set_seq = c.seq;
		}
	}
}

K_THREAD_DEFINE(motor_tid, STACK_SZ, motor_thread, NULL, NULL, NULL, PRIO_MOTOR, 0,
		SYS_FOREVER_MS);

int motor_init(void)
{
	for (int i = 0; i < 4; i++) {
		gpio_pin_configure_dt(&in[i], GPIO_OUTPUT_INACTIVE);
	}
	gpio_pin_configure_dt(&tp_pwm_set, GPIO_OUTPUT_INACTIVE);
	motor_brake();   /* start in a safe, stopped state */
	k_thread_start(motor_tid);
	return 0;
}

/* ---------- low-level outputs ---------- */

void motor_forward(uint16_t duty)
{
	if (duty > DUTY_MAX) {
		duty = DUTY_MAX;
	}
	for (int s = 0; s < 2; s++) {
		gpio_pin_set_dt(&in[2 * s],     invert[s] ? 0 : 1);
		gpio_pin_set_dt(&in[2 * s + 1], invert[s] ? 1 : 0);
		pwm_set_pulse_dt(&en[s], (uint32_t)((uint64_t)en[s].period * duty / DUTY_MAX));
	}
}

void motor_brake(void)
{
	/* 1. Both IN pins LOW: IN1 == IN2, so the bridge shorts the motor.
	 *    Set these FIRST so the motor stops being driven immediately. */
	for (int i = 0; i < 4; i++) {
		gpio_pin_set_dt(&in[i], 0);
	}
	/* 2. EN held fully ON (100 %, no PWM). With EN low the motor would
	 *    coast instead; with PWM it would alternate brake/coast. */
	for (int s = 0; s < 2; s++) {
		pwm_set_pulse_dt(&en[s], en[s].period);
	}
}