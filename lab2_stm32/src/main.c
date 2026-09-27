/*
 * Task 3.1: throttle -> target wheel velocity, held by a PI controller
 * on the average of the two encoders.
 *
 * Serial console commands (115200 baud):
 *   thr <0..1000>        set throttle (closed loop)
 *   open <0..1000>       fixed duty, no control (to measure top speed)
 *   gains <kp> <ki>      change PI gains live
 *   status               print target, velocities, duty, counts
 */
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include "config.h"
#include "encoder.h"
#include "motor.h"

/* Inputs from the shell thread. */
static atomic_t throttle;          /* 0..THROTTLE_MAX */
static atomic_t open_duty = ATOMIC_INIT(-1);   /* -1 = closed loop */
static float kp = KP, ki = KI;
static struct k_spinlock lock;     /* protects kp, ki and the status below */

/* Status for printing. */
static float st_target, st_vel[2], st_avg;
static int32_t st_counts[2];
static uint16_t st_duty;

/* Throttle -> target rpm: straight line, monotonic. */
static float throttle_to_rpm(int t)
{
	return VEL_MAX_RPM * (float)t / THROTTLE_MAX;
}

static float clampf(float x, float lo, float hi)
{
	return x < lo ? lo : (x > hi ? hi : x);
}

K_TIMER_DEFINE(control_timer, NULL, NULL);

int main(void)
{
	if (motor_init() || encoder_init()) {
		printk("init failed\n");
		return 0;
	}

	const float dt = CONTROL_PERIOD_MS / 1000.0f;
	int32_t last[2] = { encoder_read(0), encoder_read(1) };
	float integral = 0.0f;

	/* Periodic timer: fixed period, no drift. */
	k_timer_start(&control_timer, K_MSEC(CONTROL_PERIOD_MS), K_MSEC(CONTROL_PERIOD_MS));

	while (1) {
		k_timer_status_sync(&control_timer);   /* wait for next tick */

		/* 1. Velocity of each wheel from encoder counts. */
		int32_t counts[2];
		float vel[2];

		for (int s = 0; s < 2; s++) {
			counts[s] = encoder_read(s);
			vel[s] = (counts[s] - last[s]) / COUNTS_PER_REV / dt * 60.0f;  /* rpm */
			last[s] = counts[s];
		}
		float avg = (vel[0] + vel[1]) / 2.0f;

		/* 2. Target from throttle. */
		float target = throttle_to_rpm(atomic_get(&throttle));

		/* 3. PI controller -> duty. */
		k_spinlock_key_t key = k_spin_lock(&lock);
		float p = kp, i = ki;

		k_spin_unlock(&lock, key);

		uint16_t duty;
		int open = atomic_get(&open_duty);

		if (open >= 0) {
			duty = open;                     /* open loop test */
			integral = 0.0f;
		} else if (target <= 0.0f) {
			duty = 0;                        /* stopped */
			integral = 0.0f;
		} else {
			float err = target - avg;

			integral = clampf(integral + i * err * dt, 0.0f, I_LIMIT);
			duty = (uint16_t)clampf(p * err + integral, 0.0f, DUTY_MAX);
		}
		motor_forward(duty);

		/* 4. Save status for the `status` command. */
		key = k_spin_lock(&lock);
		st_target = target;
		st_vel[0] = vel[0];
		st_vel[1] = vel[1];
		st_avg = avg;
		st_counts[0] = counts[0];
		st_counts[1] = counts[1];
		st_duty = duty;
		k_spin_unlock(&lock, key);
	}
}

/* ---------------- Serial commands ---------------- */

static int cmd_thr(const struct shell *sh, size_t argc, char **argv)
{
	int t = atoi(argv[1]);

	if (t < 0 || t > THROTTLE_MAX) {
		shell_error(sh, "range 0..%d", THROTTLE_MAX);
		return -EINVAL;
	}
	atomic_set(&open_duty, -1);
	atomic_set(&throttle, t);
	return 0;
}

static int cmd_open(const struct shell *sh, size_t argc, char **argv)
{
	int d = atoi(argv[1]);

	if (d < 0 || d > DUTY_MAX) {
		shell_error(sh, "range 0..%d", DUTY_MAX);
		return -EINVAL;
	}
	atomic_set(&open_duty, d);
	return 0;
}

static int cmd_gains(const struct shell *sh, size_t argc, char **argv)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	kp = strtof(argv[1], NULL);
	ki = strtof(argv[2], NULL);
	k_spin_unlock(&lock, key);
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	shell_print(sh, "target=%.1f  vL=%.1f  vR=%.1f  avg=%.1f rpm  duty=%u  countL=%d  countR=%d",
		    (double)st_target, (double)st_vel[0], (double)st_vel[1],
		    (double)st_avg, st_duty, st_counts[0], st_counts[1]);
	k_spin_unlock(&lock, key);
	return 0;
}

SHELL_CMD_ARG_REGISTER(thr, NULL, "Throttle 0..1000", cmd_thr, 2, 0);
SHELL_CMD_ARG_REGISTER(open, NULL, "Open-loop duty 0..1000", cmd_open, 2, 0);
SHELL_CMD_ARG_REGISTER(gains, NULL, "PI gains: <kp> <ki>", cmd_gains, 3, 0);
SHELL_CMD_ARG_REGISTER(status, NULL, "Print status", cmd_status, 1, 0);

/* TODO 3.2: brake pedal -> motor_brake(), brake beats throttle */
