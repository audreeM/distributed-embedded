/*
 * Blinkers on plain GPIO, toggled from one kernel timer.
 *
 * They used to be hardware PWM on TIM2, but TIM2 also drives the servo, and
 * all channels of a timer share one period (servo 20 ms vs blink 0.5-1 s), so
 * the two can't share it. A k_timer gives the same exact rate from the
 * system clock, and because one callback writes all four pins, front and
 * rear on a side switch within microseconds of each other.
 *
 * Periods are still passed in nanoseconds via PWM_HZ(), as before.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h> /* only for the PWM_HZ() macro */
#include "blinker.h"
#include "config.h"
#include "link.h"
#include "sys_state.h"

#define NUM_LEDS 4
#define BLINK_TICK_MS 250 /* half of the fastest period (2 Hz hazards) */

// 0-1 = left (front, rear), 2-3 = right (front, rear)
static const struct gpio_dt_spec leds[NUM_LEDS] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(blink_led_0), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(blink_led_1), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(blink_led_2), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(blink_led_3), gpios),
};

static struct k_spinlock lock;
static uint32_t period_ms[NUM_LEDS]; // 0 = off
static uint32_t ticks;               // ticks since the pattern last changed

// every 250 ms: each LED is on during the first half of its period
static void blink_tick(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t elapsed = ticks++ * BLINK_TICK_MS;

	for (int i = 0; i < NUM_LEDS; i++) {
		uint32_t p = period_ms[i];

		gpio_pin_set_dt(&leds[i], p != 0 && (elapsed % p) < p / 2);
	}
	k_spin_unlock(&lock, key);
}

K_TIMER_DEFINE(blink_timer, blink_tick, NULL);

// set LEDs first..last-1 to a period (ns, from PWM_HZ) and restart the pattern
static void set_range(int first, int last, uint32_t period_ns)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	for (int i = first; i < last; i++) {
		period_ms[i] = period_ns / 1000000U;
	}
	ticks = 0;
	k_spin_unlock(&lock, key);

	// restart so a new pattern turns on right away (R2.4: within 100 ms)
	blink_tick(NULL);
	k_timer_start(&blink_timer, K_MSEC(BLINK_TICK_MS), K_MSEC(BLINK_TICK_MS));
}

int set_all_blinkers(uint32_t period) {
	set_range(0, NUM_LEDS, period);
	return 0;
}

int set_left_blinkers(uint32_t period) {
	set_range(0, NUM_LEDS/2, period);
	return 0;
}

int set_right_blinkers(uint32_t period) {
	set_range(NUM_LEDS/2, NUM_LEDS, period);
	return 0;
}

int blinker_init(void)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		if (!gpio_is_ready_dt(&leds[i]) ||
		    gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE) < 0) {
			return -ENODEV;
		}
	}
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
		set_left_blinkers(0); // other side off (one side cancels the other)
		set_right_blinkers(PWM_HZ(1));
		return;
	} else if (state == LEFT_NPT || state == LEFT_PT) {
		set_right_blinkers(0); // other side off
		set_left_blinkers(PWM_HZ(1));
		return;
	} else {
		set_all_blinkers(PWM_HZ(2));
		return;
	}
}

/* ------------------------------------------------------------ thread */

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

K_TIMER_DEFINE(blinker_timer, NULL, NULL);

// every 20 ms: turn the latest command into a link_event_t and step the state machine
static void blinker_thread(void *p1, void *p2, void *p3)
{
	struct link_cmd c;
	state_t state = HAZARD; // we boot in ERROR (POWERUP), so hazards
	bool prev_error = true;
	uint8_t prev_buttons = 0;
	wheel_event_t wheel = MID_RANGE;

	if (blinker_init() < 0) {
		printk("blinker GPIO not ready\n");
		return;
	}

	k_timer_start(&blinker_timer, K_MSEC(BLINK_PERIOD_MS), K_MSEC(BLINK_PERIOD_MS));

	for (;;) {
		k_timer_status_sync(&blinker_timer);

		link_get_cmd(&c);
		bool error = sys_state_is_error();
		uint8_t buttons = c.valid ? c.buttons : 0;
		// a press is the moment the button goes down, not while it is held
		uint8_t pressed = buttons & ~prev_buttons;
		link_event_t ev;

		// error: only report the moment we enter or leave it
		if (error && !prev_error) {
			ev.err = ENTER_ERR;
		} else if (!error && prev_error) {
			ev.err = EXIT_ERR;
		} else {
			ev.err = NO_ERR;
		}

		if (pressed & LP_BTN_LEFT) {
			ev.button = LEFT_PRESS;
		} else if (pressed & LP_BTN_RIGHT) {
			ev.button = RIGHT_PRESS;
		} else {
			ev.button = NO_PRESS;
		}

		wheel = wheel_range(c.valid ? c.steer : 0, wheel);
		ev.wheel = wheel;

		state_t next = determine_next_state(state, ev);

		// only touch the LEDs when the state changes, so the blink keeps its phase
		if (next != state) {
			update_outputs(next);
			state = next;
		}

		prev_error = error;
		prev_buttons = buttons;
	}
}

K_THREAD_DEFINE(blinker_tid, STACK_SZ, blinker_thread, NULL, NULL, NULL, PRIO_BLINK, 0,
		SYS_FOREVER_MS);
