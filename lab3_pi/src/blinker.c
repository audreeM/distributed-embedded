#include <string.h>

#include "blinker.h"
#include "config.h"
#include "util.h"

void blinker_init(struct blinker *b)
{
	memset(b, 0, sizeof(*b));
	b->state = HAZARD; // we boot in ERROR (POWERUP), so hazards
	b->prev_error = true;
	b->wheel = MID_RANGE;
	b->mode = CAN_BLINK_HAZARD;
}

/* Copied from lab2_stm32/src/blinker.c, unchanged. */
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

/* Copied from lab2_stm32/src/blinker.c, unchanged. */
// which wheel range steer is in, with hysteresis around the thresholds
static wheel_event_t wheel_range(int steer, wheel_event_t prev)
{
	if (steer >= BLINK_ARM_STEER) {
		return RIGHT_RANGE;
	}
	if (steer <= -BLINK_ARM_STEER) {
		return LEFT_RANGE;
	}
	// between the thresholds: stay where we were until we're back inside CANCEL
	if (prev == RIGHT_RANGE && steer > BLINK_CANCEL_STEER) {
		return RIGHT_RANGE;
	}
	if (prev == LEFT_RANGE && steer < -BLINK_CANCEL_STEER) {
		return LEFT_RANGE;
	}
	return MID_RANGE;
}

/* Replaces update_outputs() from Lab 2: the cockpit no longer drives lamps,
 * it only says which lamps should blink.
 */
static uint8_t state_to_mode(state_t state)
{
	switch (state) {
	case HAZARD:
		return CAN_BLINK_HAZARD;
	case RIGHT_NPT:
	case RIGHT_PT:
		return CAN_BLINK_RIGHT;
	case LEFT_NPT:
	case LEFT_PT:
		return CAN_BLINK_LEFT;
	default:
		return CAN_BLINK_OFF;
	}
}

bool blinker_update(struct blinker *b, const struct blinker_inputs *in, uint64_t now_us)
{
	/* Copied from the loop body of blinker_thread() in lab2_stm32/src/blinker.c:
	 * turn the latest inputs into a link_event_t and step the state machine.
	 */
	bool error = in->hazard;
	link_event_t ev;

	// error: only report the moment we enter or leave it
	if (error && !b->prev_error) {
		ev.err = ENTER_ERR;
	} else if (!error && b->prev_error) {
		ev.err = EXIT_ERR;
	} else {
		ev.err = NO_ERR;
	}

	// a press is the moment the button goes down, not while it is held
	if (in->left_btn && !b->prev_left) {
		ev.button = LEFT_PRESS;
	} else if (in->right_btn && !b->prev_right) {
		ev.button = RIGHT_PRESS;
	} else {
		ev.button = NO_PRESS;
	}

	b->wheel = wheel_range(in->steer, b->wheel);
	ev.wheel = b->wheel;

	state_t next = determine_next_state(b->state, ev);

	b->prev_error = error;
	b->prev_left = in->left_btn;
	b->prev_right = in->right_btn;

	bool changed = (next != b->state);

	if (changed) {
		b->state = next;
		b->mode = state_to_mode(next);
	}

	/*
	 * TODO(cockpit): the blink clock (new in Lab 3; Lab 2 used a k_timer on
	 * the STM32 that toggled the LEDs directly).
	 *   - on `changed`: phase = 1 (lamps on now), next_edge_us = now_us +
	 *     half period, and return true so the new mode goes out right away
	 *     (R2.4: blinker starts within 100 ms of the UDP update).
	 *   - while mode != OFF: when now_us >= next_edge_us, flip phase,
	 *     next_edge_us += half period (BLINK_HALF_MS, or HAZARD_HALF_MS for
	 *     HAZARD) and return true. Adding to next_edge_us instead of
	 *     now_us keeps the rate from drifting: 1 Hz +/- 10 %, 50 % duty.
	 *   - mode OFF: phase = 0; the one frame sent on `changed` is enough.
	 *   - never return true twice within BLINK_MIN_GAP_MS of last_sent_us
	 *     (the BLINK minimum interarrival the bus analysis assumes).
	 *
	 * TODO(cockpit): decide what Appendix A's last blinker row should do
	 * (right blinker on, wheel past right threshold, press right, then wheel
	 * swings to the left threshold). determine_next_state() gives
	 * RIGHT_PT -> OFF on the press; check that is the behavior you document.
	 */
	(void)now_us;
	return false;
}
