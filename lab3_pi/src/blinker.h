/*
 * Blink master (BLINK_TX). The cockpit owns the blinker state and the blink
 * timing; N2 and N3 only drive their lamps from the BLINK frames it sends
 * (S4, R2.4). Self-cancel lives here because only the cockpit has both the
 * buttons and the wheel angle.
 */
#ifndef BLINKER_H
#define BLINKER_H

#include <stdbool.h>
#include <stdint.h>

#include "can_catalog.h"

/* Copied from lab2_stm32/include/blinker.h. */
// Typedefs to define blinker states and events corresponding to error,
// wheel, and button inputs
typedef enum
{
	HAZARD,
	OFF,
	RIGHT_NPT,
	RIGHT_PT,
	LEFT_NPT,
	LEFT_PT
} state_t;
typedef enum
{
	NO_ERR,
	ENTER_ERR,
	EXIT_ERR
} err_event_t;
typedef enum
{
	LEFT_RANGE,
	MID_RANGE,
	RIGHT_RANGE
} wheel_event_t;
typedef enum
{
	NO_PRESS,
	RIGHT_PRESS,
	LEFT_PRESS
} button_event_t;

// Struct to pack inputs into 1 event
typedef struct
{
	err_event_t err;
	wheel_event_t wheel;
	button_event_t button;
} link_event_t;

struct blinker
{
	uint8_t mode;		   /* CAN_BLINK_*, what the lamps are showing */
	uint8_t phase;		   /* 1 = lamps of `mode` on right now, 0 = off */
	uint64_t next_edge_us; /* when the lamps toggle next */
	uint64_t last_sent_us; /* last BLINK frame, for BLINK_MIN_GAP_MS */

	/* Lab 2 kept these as locals in blinker_thread(); here they live in the
	 * struct because blinker_update() is called once per tick instead.
	 */
	state_t state;
	bool prev_error;
	bool prev_left, prev_right;
	wheel_event_t wheel;
	bool pending; /* the STM learns to stop blinking on a BLINK frame, it may stop blinking for other reasons*/
};

struct blinker_inputs
{
	bool left_btn;	/* left turn button pressed right now */
	bool right_btn; /* right turn button pressed right now */
	int steer;		/* -100 .. +100 */
	bool hazard;	/* the cockpit is not NORMAL: show hazards (R3, S4) */
};

void blinker_init(struct blinker *b);
state_t determine_next_state(state_t state, link_event_t event);

/*
 * Called every TICK_MS. Returns true when a BLINK frame carrying b->mode and
 * b->phase must go out now; main.c packs and sends it.
 */
bool blinker_update(struct blinker *b, const struct blinker_inputs *in, uint64_t now_us);

#endif /* BLINKER_H */
