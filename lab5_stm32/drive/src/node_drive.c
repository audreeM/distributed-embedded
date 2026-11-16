/* N3 drivetrain: the node-specific hooks (see shared/include/node.h). */
#include "motor.h"
#include "node.h"
#include "pid.h"

const char *const node_name = "drivetrain";

int node_outputs_init(void)
{
	motor_init(); /* high priority motor thread, starts braked */
	pid_init();   /* lower priority PID thread */
	return 0;
}

void node_on_ctrl(const struct can_ctrl *c)
{
	motor_on_ctrl(c);
}

void node_kick_outputs(void)
{
	motor_kick();
}

int node_tx_init(void)
{
	return 0; /* N3 only sends its heartbeat (shared hb_tx.c) */
}
