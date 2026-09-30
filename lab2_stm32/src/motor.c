/**
 * @file motor.c
 * @brief motor_thread runs every 10 ms: reads the encoders and updates the motors
 * 
 * L298N: EN pins get PWM (speed), IN pins set direction.
 *
 * L298N truth table, per motor:
 *   EN low              -> coast (motor free-wheels)
 *   EN high, IN1 != IN2 -> drive (which one is high sets direction)
 *   EN high, IN1 == IN2 -> dynamic brake (motor terminals shorted together)
 */
#include "config.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "motor.h"
#include "encoder.h"
#include "link.h"
#include "sys_state.h"

static float clampf(float x, float lo, float hi)
{
	return x < lo ? lo : (x > hi ? hi : x);
}

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

K_TIMER_DEFINE(motor_timer, NULL, NULL);

static void motor_thread(void *p1, void *p2, void *p3)
{
	if (motor_init() || encoder_init()) {
		printk("motor/encoder init failed\n");
		sys_state_set_fault(LP_FAULT_SELF_TEST);   // or whichever fault bit your team uses
		return;
	}

	const float dt = CONTROL_PERIOD_MS / 1000.0f;
	int32_t last[2] = { encoder_read(0), encoder_read(1) };
	float integral = 0.0f;
	struct link_cmd c;

	// periodic timer instead of k_msleep so the period doesn't drift by the loop's run time
	k_timer_start(&motor_timer, K_MSEC(CONTROL_PERIOD_MS), K_MSEC(CONTROL_PERIOD_MS));

	for (;;) {
		k_timer_status_sync(&motor_timer);

		// wheel velocity (rpm) from encoder counts since the last tick
		float vel[2];

		for (int s = 0; s < 2; s++) {
			int32_t now = encoder_read(s);

			vel[s] = (now - last[s]) / COUNTS_PER_REV / dt * 60.0f;
			last[s] = now;
		}
		float avg = (vel[0] + vel[1]) / 2.0f;

		link_get_cmd(&c);

		// safe state on fault or no command yet; brake beats throttle,
		// so this is checked before throttle is even looked at
		if (sys_state_is_error() || !c.valid || c.brake > 0) {
			integral = 0.0f;   // don't wind up while stopped
			motor_brake();
			continue;
		}

		// throttle 0..100 % -> target rpm
		float target = VEL_MAX_RPM * c.throttle / 100.0f;

		if (target <= 0.0f) {
			integral = 0.0f;
			motor_forward(0);   // coast
			continue;
		}

		// PI on the average wheel speed
		float err = target - avg;

		integral = clampf(integral + KI * err * dt, 0.0f, I_LIMIT);
		uint16_t duty = (uint16_t)clampf(KP * err + integral, 0.0f, DUTY_MAX);

		// TODO: remove later
		static int n;
		if (++n % 20 == 0) {   // every 200 ms
			printk("tgt=%.1f vL=%.1f vR=%.1f duty=%u\n",
				(double)target, (double)vel[0], (double)vel[1], duty);
		}
		// print end
		motor_forward(duty);
	}
}


K_THREAD_DEFINE(motor_tid, STACK_SZ, motor_thread, NULL, NULL, NULL, PRIO_MOTOR, 0, SYS_FOREVER_MS);

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
