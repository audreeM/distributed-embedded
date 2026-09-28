#include "config.h"
#include "state.h"
#include "link.h"

int main(void)
{
	// TODO: state_init() - drive outputs to the safe state and enter ERROR
	// before any thread is allowed to touch an actuator
	state_init();

	// wakes up when a frame arrives, serves brake, throttle, servo, and button presses
	k_thread_start(cmd_tid);

	// every 10 ms reads the encoders and updates the motors
	k_thread_start(motor_tid);

	// every 20 ms sends the status frame (state + current) back to the Pi
	// related to part 2
	k_thread_start(status_tid);

	// every 10 ms reads the three current sensors
	k_thread_start(sense_tid);

	return 0;
}
