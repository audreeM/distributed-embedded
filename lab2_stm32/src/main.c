#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include "blinker.h"


int count = 0; // Used for testing purposes to control state transitions

link_event_t read_event() 
{
	count++;
	link_event_t test_event;
	k_sleep(K_SECONDS(5));
	if (count == 1)
		test_event = (link_event_t){EXIT_ERR, MID_RANGE, NO_PRESS};
	else if (count == 2)
		test_event = (link_event_t){NO_ERR, MID_RANGE, RIGHT_PRESS};
	else if (count == 3)
		test_event = (link_event_t){NO_ERR, RIGHT_RANGE, NO_PRESS};
	else if (count == 4)
		test_event = (link_event_t){NO_ERR, MID_RANGE, NO_PRESS};
	else if (count == 5)
		test_event = (link_event_t){NO_ERR, MID_RANGE, LEFT_PRESS};
	else if (count == 6)
		test_event = (link_event_t){NO_ERR, LEFT_RANGE, NO_PRESS};
	else if (count == 7)
		test_event = (link_event_t){NO_ERR, MID_RANGE, NO_PRESS};
	else
		test_event = (link_event_t){ENTER_ERR, MID_RANGE, NO_PRESS};
	return test_event;
}

int main(void)
{
	int status = blinker_init();
	
	if (status) {
		return 1;
	}

	printk("Moving to main loop\n");

	state_t state = HAZARD;
	while(1) 
	{
		link_event_t link_event = read_event();

		state_t new_state = determine_next_state(state, link_event);
		if (new_state != state) {
			state = new_state;
			printk("State: %d\n", state);
		} else {
			continue;
		}
		update_outputs(state);
	}
}