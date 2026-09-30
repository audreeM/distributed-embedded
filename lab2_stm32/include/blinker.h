#ifndef BLINKER_H_
#define BLINKER_H_
#include <stdint.h>

// Typedefs to define blinker states and events corresponding to error,
// wheel, and button inputs
typedef enum {HAZARD, OFF, RIGHT_NPT, RIGHT_PT, LEFT_NPT, LEFT_PT} state_t;
typedef enum {NO_ERR, ENTER_ERR, EXIT_ERR} err_event_t;
typedef enum {LEFT_RANGE, MID_RANGE, RIGHT_RANGE} wheel_event_t;
typedef enum {NO_PRESS, RIGHT_PRESS, LEFT_PRESS} button_event_t;

// Struct to pack inputs into 1 event
typedef struct {
	err_event_t err;
	wheel_event_t wheel;
	button_event_t button;
} link_event_t;

int set_all_blinkers(uint32_t period);
int set_left_blinkers(uint32_t period);
int set_right_blinkers(uint32_t period);
int blinker_init(void);
state_t determine_next_state(state_t state, link_event_t event);
void update_outputs(state_t state);

#endif /* BLINKER_H_ */