#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "link.h"
#include "sys_state.h"

/* Faults that clear themselves once good commands flow again. SELF_TEST is
 * left out on purpose: only a double press clears it.
 */
#define RECOVERABLE (LP_FAULT_POWERUP | LP_FAULT_LINK_LOST | LP_FAULT_BAD_CMD)

/* The whole fault state is one atomic word. It is written from the watchdog
 * ISR and the link_rx thread and read everywhere, and atomics make every
 * read and read-modify-write indivisible without disabling interrupts.
 * Starting with POWERUP set means the zone boots into ERROR.
 */
static atomic_t faults = ATOMIC_INIT(LP_FAULT_POWERUP);
static atomic_t good_count; /* consecutive good commands since the last fault */

uint32_t sys_state_faults(void)
{
	return (uint32_t)atomic_get(&faults);
}

bool sys_state_is_error(void)
{
	return atomic_get(&faults) != 0;
}

void sys_state_set_fault(uint32_t bits)
{
	atomic_val_t old = atomic_or(&faults, bits);

	/* Any new fault restarts the "N good commands" recovery count. */
	atomic_clear(&good_count);
	/* Only wake the control thread if something actually changed. */
	if ((old | bits) != old) {
		link_notify();
	}
}

void sys_state_clear_fault(uint32_t bits)
{
	atomic_val_t old = atomic_and(&faults, ~bits);

	if ((old & ~bits) != old) {
		link_notify();
	}
}

/* Requiring 2 in a row (not 1) stops a single stray valid frame from
 * releasing the brakes.
 */
void sys_state_on_good_cmd(void)
{
	if (atomic_inc(&good_count) + 1 >= SYS_STATE_GOOD_TO_CLEAR) {
		sys_state_clear_fault(RECOVERABLE);
	}
}

void sys_state_on_bad_cmd(void)
{
	sys_state_set_fault(LP_FAULT_BAD_CMD);
}

/*
 * Single press while not in self-test -> enter self-test fault immediately.
 * While in self-test, two presses within SELFTEST_DOUBLE_MS clear it; the
 * press that caused the failure does not count toward that.
 * Presses are rising edges of the sampled button bit, with a leading-edge
 * lockout so contact bounce spread across two command frames is one press.
 */
void sys_state_on_buttons(uint8_t buttons, int64_t now_ms)
{
	static bool prev_pressed;
	static int64_t lockout_until_ms;
	static int64_t first_press_ms = -1;

	/* The Pi sends the button LEVEL in every frame, not press events, so a
	 * lost frame can't lose a press. We turn the level into press edges
	 * here.
	 */
	bool pressed = (buttons & LP_BTN_TEST) != 0;
	bool edge = pressed && !prev_pressed;

	prev_pressed = pressed;
	if (!edge || now_ms < lockout_until_ms) {
		return;
	}
	lockout_until_ms = now_ms + SELFTEST_LOCKOUT_MS;

	/* Normal (or another fault): one press = local fail, right away. */
	if (!(sys_state_faults() & LP_FAULT_SELF_TEST)) {
		sys_state_set_fault(LP_FAULT_SELF_TEST);
		first_press_ms = -1;
		return;
	}

	/* Already in self-test: the second press within the window restores;
	 * otherwise this press becomes the first of a possible double press.
	 */
	if (first_press_ms >= 0 && now_ms - first_press_ms <= SELFTEST_DOUBLE_MS) {
		sys_state_clear_fault(LP_FAULT_SELF_TEST);
		first_press_ms = -1;
	} else {
		first_press_ms = now_ms;
	}
}
