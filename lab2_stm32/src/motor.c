/**
 * @file motor.c
 * @brief motor_thread runs the PI loop every CONTROL_PERIOD_MS, and also wakes
 *        on every new command / fault change (link_wait) to brake at once
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
#include "cli.h"

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

// PWM_SET test point (PC9): toggled right after a new duty is written, so the
// scope shows CMD_RX -> PWM_SET = software response time for R2.1
static const struct gpio_dt_spec tp_pwm_set = GPIO_DT_SPEC_GET(USER, pwmset_gpios);

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
	float ramp = 0.0f;                  // target that slides toward the pedal
	float vel_f = 0.0f, prev_vel_f = 0.0f;  // filtered speed, for P/I and D
	struct link_cmd c;

	// Absolute tick deadline for the next control step, so the period doesn't
	// drift by the loop's run time or by how often a command wakes us early.
	const int64_t period_ticks = k_ms_to_ticks_ceil64(CONTROL_PERIOD_MS);
	int64_t next = k_uptime_ticks() + period_ticks;

	for (;;) {
		// Sleep until the next control tick OR a new command / fault change,
		// whichever comes first. This thread is the only link_wait() caller.
		if (link_wait(K_TIMEOUT_ABS_TICKS(next))) {
			// Woken early: act on brake / fail-safe now instead of at the
			// next tick (R2.2, 2 ms). Throttle waits for the tick, so the
			// PI loop keeps its fixed dt.
			link_get_cmd(&c);
			if (sys_state_is_error() || !c.valid || c.brake > 0) {
				integral = 0.0f;
				ramp = 0.0f;
				motor_brake();
			}
			continue;
		}
		next += period_ticks;

		// wheel velocity (rpm) from encoder counts since the last tick
		float vel[2];

		for (int s = 0; s < 2; s++) {
			int32_t now = encoder_read(s);

			vel[s] = (now - last[s]) / COUNTS_PER_REV / dt * 60.0f;
			last[s] = now;
		}
		float avg = (vel[0] + vel[1]) / 2.0f;
		// low-pass the speed: at a 2 ms period one encoder count is ~23 rpm,
		// so the raw average jumps around too much to control on
		vel_f += VEL_ALPHA * (avg - vel_f);
		float accel = (vel_f - prev_vel_f) / dt;   // for the D term
		prev_vel_f = vel_f;

		link_get_cmd(&c);

		// safe state on fault or no command yet; brake beats throttle,
		// so this is checked before throttle is even looked at
		if (sys_state_is_error() || !c.valid || c.brake > 0) {
			integral = 0.0f;   // don't wind up while stopped
			ramp = 0.0f;       // wheels stop, so start the next ramp from 0
			motor_brake();
			continue;
		}

		// throttle 0..100 % -> goal rpm, then move the target toward the goal
		// by at most ACCEL_RPM_PER_S, so speed changes gradually
		float goal = VEL_MAX_RPM * c.throttle / 100.0f;
		float step = ACCEL_RPM_PER_S * dt;

		ramp = (goal > ramp) ? MIN(ramp + step, goal) : MAX(ramp - step, goal);
		float target = ramp;

		if (target <= 0.0f) {
			integral = 0.0f;
			motor_forward(0);   // coast
			continue;
		}

		// PID on the filtered average wheel speed. D acts on the measured
		// speed (not the error), so a pedal change doesn't cause a kick.
		float err = target - vel_f;

		// feed-forward: the duty this speed roughly needs, so the wheels
		// start at once; PID only corrects the leftover error
		float ff = FF_OFFSET + FF_SLOPE * target;

		integral = clampf(integral + KI * err * dt, -I_LIMIT, I_LIMIT);
		uint16_t duty = (uint16_t)clampf(ff + KP * err + integral - KD * accel,
						 0.0f, DUTY_MAX);

		// TODO: remove later
		static int n;
		if (++n % 20 == 0 && cli_log_enabled()) {   // every 200 ms, `log on`
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
	gpio_pin_configure_dt(&tp_pwm_set, GPIO_OUTPUT_INACTIVE);
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
	// both duties are written: mark it on the PWM_SET test point
	gpio_pin_toggle_dt(&tp_pwm_set);
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