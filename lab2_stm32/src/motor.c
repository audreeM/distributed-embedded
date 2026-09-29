/*
 * L298N: EN pins get PWM (speed), IN pins set direction.
 *
 * L298N truth table, per motor:
 *   EN low              -> coast (motor free-wheels)
 *   EN high, IN1 != IN2 -> drive (which one is high sets direction)
 *   EN high, IN1 == IN2 -> dynamic brake (motor terminals shorted together)
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "motor.h"
#include "config.h"

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

int motor_init(void)
{
	for (int i = 0; i < 4; i++) {
		gpio_pin_configure_dt(&in[i], GPIO_OUTPUT_INACTIVE);
	}
	motor_brake();   /* start in a safe, stopped state */
	return 0;
}

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
	/* TODO checkoff: toggle PWM_SET test point here */
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
