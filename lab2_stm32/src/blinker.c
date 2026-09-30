#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include "blinker.h"

#define NUM_LEDS 4

static const struct pwm_dt_spec leds[NUM_LEDS] = 
{
	PWM_DT_SPEC_GET(DT_NODELABEL(blink_led_0)),
	PWM_DT_SPEC_GET(DT_NODELABEL(blink_led_1)),
	PWM_DT_SPEC_GET(DT_NODELABEL(blink_led_2)),
	PWM_DT_SPEC_GET(DT_NODELABEL(blink_led_3))	
};

int set_all_blinkers(uint32_t period) {
	for (int i = 0; i < NUM_LEDS; i++) {
		pwm_set_dt(&leds[i], period, period / 2);
	}
	return 0;
}

int set_left_blinkers(uint32_t period) {
	for (int i = 0; i < NUM_LEDS/2; i++) {
		pwm_set_dt(&leds[i], period, period / 2);
	}
	return 0;
}

int set_right_blinkers(uint32_t period) {
	for (int i = NUM_LEDS/2; i < NUM_LEDS; i++) {
		pwm_set_dt(&leds[i], period, period / 2);
	}
	return 0;
}

int blinker_init(void)
{
	set_all_blinkers(PWM_HZ(2));
	return 0;
}

state_t determine_next_state(state_t state, link_event_t event) 
{
	switch (state)
	{
		case HAZARD:
			if (event.err == EXIT_ERR) 
				return OFF;
			break;
		case OFF:
			if (event.err == ENTER_ERR) 
				return HAZARD;
			if ((event.button == RIGHT_PRESS) && (event.wheel != RIGHT_RANGE))
				return RIGHT_NPT;
			if ((event.button == RIGHT_PRESS) && (event.wheel == RIGHT_RANGE))
				return RIGHT_PT;
			if ((event.button == LEFT_PRESS) && (event.wheel != LEFT_RANGE))
				return LEFT_NPT;
			if ((event.button == LEFT_PRESS) && (event.wheel == LEFT_RANGE))
				return LEFT_PT;
			break;
		case RIGHT_NPT:
			if (event.err == ENTER_ERR)
				return HAZARD;
			if (event.button == RIGHT_PRESS)
				return OFF;
			if (event.wheel == RIGHT_RANGE)
				return RIGHT_PT;
			if ((event.button == LEFT_PRESS) && (event.wheel != LEFT_RANGE))
				return LEFT_NPT;
			if ((event.button == LEFT_PRESS) && (event.wheel == LEFT_RANGE))
				return LEFT_PT;
			break;
		case LEFT_NPT:
			if (event.err == ENTER_ERR)
				return HAZARD;
			if (event.button == LEFT_PRESS)
				return OFF;
			if (event.wheel == LEFT_RANGE)
				return LEFT_PT;
			if ((event.button == RIGHT_PRESS) && (event.wheel != RIGHT_RANGE))
				return RIGHT_NPT;
			if ((event.button == RIGHT_PRESS) && (event.wheel == RIGHT_RANGE))
				return RIGHT_PT;
			break;
		case RIGHT_PT:
			if (event.err == ENTER_ERR)
				return HAZARD;
			if (event.button == RIGHT_PRESS || event.wheel == MID_RANGE)
				return OFF;
			if (event.button == LEFT_PRESS)
				return LEFT_NPT;
			break;
		case LEFT_PT:
			if (event.err == ENTER_ERR)
				return HAZARD;
			if (event.button == LEFT_PRESS || event.wheel == MID_RANGE)
				return OFF;
			if (event.button == RIGHT_PRESS)
				return RIGHT_NPT;
			break;
	}
	return state;
}

void update_outputs(state_t state)
{
	if (state == HAZARD) {
		set_all_blinkers(PWM_HZ(2));
		return;
	} else if (state == OFF) {
		set_all_blinkers(0);
		return;
	} else if (state == RIGHT_NPT || state == RIGHT_PT) {
		set_right_blinkers(PWM_HZ(1));
		return;
	} else if (state == LEFT_NPT || state == LEFT_PT) {
		set_left_blinkers(PWM_HZ(1));
		return;
	} else {
		set_all_blinkers(PWM_HZ(2));
		return;
	}
}
