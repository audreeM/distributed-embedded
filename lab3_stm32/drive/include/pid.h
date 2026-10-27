/**
 * @file pid.h
 * @brief Small PID controller with a feed-forward bias, an anti-windup clamp
 *        and derivative-on-measurement.
 *
 *   output = clamp(bias + kp*err + integral - kd*meas_rate, out_min, out_max)
 *
 * The D term acts on the measured rate (e.g. wheel acceleration), not on the
 * error, so a setpoint change doesn't cause a derivative kick.
 *
 * No kernel or hardware dependencies, so it can be unit-tested on the host.
 * Not thread-safe: each instance belongs to one thread.
 */
#ifndef PID_H_
#define PID_H_

struct pid_cfg {
	float kp;
	float ki;
	float kd;
	float i_limit;   /* integral term is clamped to +/- i_limit */
	float out_min;
	float out_max;
};

struct pid {
	struct pid_cfg cfg;
	float integral;
};

int pid_init(void);

/** Clear the integral (call while stopped so it can't wind up). */
void pid_reset(struct pid *p);

/** Output for the current state WITHOUT integrating (integral is frozen). */
float pid_output(const struct pid *p, float err, float meas_rate, float bias);

/** Integrate err over dt, then return the new output. */
float pid_update(struct pid *p, float err, float meas_rate, float bias, float dt);

#endif /* PID_H_ */