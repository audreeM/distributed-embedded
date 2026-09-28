/**
 * @file cmd.c
 * @brief cmd_thread wakes up when frame arrives and acts on it (for brake, throttle, servo, button presses)
 */
#include "cmd.h"

// TO DO: update with actual frame receiving code. this just pretends
// frame is arriving every 10 ms
int link_recv(struct frame *f, k_timeout_t timeout) {
	static uint8_t seq;
	k_msleep(10);
	*f = (struct frame) {.id = ID_DRIVE, .len = 4, .d = {seq++, 50}};
	return 0;
}

static void cmd_thread(void *p1, void *p2, void *p3)
{
	struct frame f;

	for (;;) {
		link_recv(&f, K_FOREVER); 

		if (!frame_valid(&f)) {
			state_fail(R_BAD_CMD);
			continue;
		}
		state_frame_ok();  

		switch (f.id) {
		case ID_BRAKE:   drivetrain_set_brake(f.d[1]);       break;
		case ID_DRIVE:   drivetrain_set_throttle(f.d[1]);    break;
		// TO DO: does the steering even work this way
		case ID_STEER:   steering_set((int8_t)f.d[1]);       break;
		case ID_BLINKER: 
		case ID_BUTTONS:  break;
		}
	}
}

K_THREAD_DEFINE(cmd_tid, STACK_SZ, cmd_thread, NULL, NULL, NULL, PRIO_CMD, 0, SYS_FOREVER_MS);
