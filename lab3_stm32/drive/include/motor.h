#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "canbus.h"

/* Latest accepted CTRL command, shared between dispatch, motor and PID. */
struct motor_cmd {
	bool valid;        /* at least one good CTRL received */
	uint8_t throttle;  /* 0..100 % */
	uint8_t brake;     /* > 0 -> brake */
	uint8_t seq;       /* local counter, bumps on every new CTRL (PWM_SET test point) */
};

/* Configure pins, brake, and start the (high priority) motor thread. */
int motor_init(void);

/* Called from the dispatch thread on every valid CTRL frame. */
void motor_on_ctrl(const struct can_ctrl *c);

/* Called from zone.c on a state change: wake the motor thread to re-evaluate now. */
void motor_kick(void);

/* Called from the PID thread: publish the requested duty (0 = coast). */
void motor_set_duty(uint16_t duty);

/* Snapshot of the latest command (safe from any thread). */
void motor_get_cmd(struct motor_cmd *out);

/* Low-level outputs; only the motor thread (and motor_init) calls these. */
void motor_forward(uint16_t duty);
void motor_brake(void);

#endif /* MOTOR_H */