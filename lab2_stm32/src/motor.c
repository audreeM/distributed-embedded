/*
 * L298N: EN pins get PWM (speed), IN pins set direction.
 * Forward = IN1 high, IN2 low (swapped if the motor is mirrored).
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
	motor_forward(0);
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
}

/* TODO 3.2: void motor_brake(void) - dynamic braking */
