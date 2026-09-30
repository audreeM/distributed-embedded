/**
 * @file cmd.c
 * @brief cmd_thread wakes up when frame arrives and acts on it (for brake, throttle, servo, button presses)
 */
#include "cmd.h"

static void cmd_thread(void *p1, void *p2, void *p3)
{
	struct link_cmd c;

	for (;;) {
		link_wait(K_FOREVER);

		if (sys_state_is_error()) {
			continue;
		}
		link_get_cmd(&c);

		// TO DO: read the frame to call other functions
	}
}

K_THREAD_DEFINE(cmd_tid, STACK_SZ, cmd_thread, NULL, NULL, NULL, PRIO_CMD, 0, SYS_FOREVER_MS);
