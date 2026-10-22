#include <string.h>

#include "config.h"
#include "util.h"
#include "zone.h"

void zone_init(struct zone *z)
{
	memset(z, 0, sizeof(*z));
	z->state = ZONE_ERROR; /* R3: every zone powers up in the error state */
}

/*
 * Copied from lab2_stm32/src/sys_state.c (sys_state_on_buttons()). In Lab 2
 * the self-test was the LP_FAULT_SELF_TEST bit; here it is z->self_test, and
 * failing also queues the FAULT frame so the other zones hear about it.
 *
 * Single press while not in self-test -> enter self-test fault immediately.
 * While in self-test, two presses within SELFTEST_DOUBLE_MS clear it; the
 * press that caused the failure does not count toward that.
 * Presses are rising edges of the sampled button bit, with a leading-edge
 * lockout so contact bounce spread across two command frames is one press.
 */
static void on_test_button(struct zone *z, bool pressed, int64_t now_ms)
{
	static bool prev_pressed;
	static int64_t lockout_until_ms;
	static int64_t first_press_ms = -1;

	/* The proxy sends the button LEVEL in every packet, not press events, so
	 * a lost packet can't lose a press. We turn the level into press edges
	 * here.
	 */
	bool edge = pressed && !prev_pressed;

	prev_pressed = pressed;
	if (!edge || now_ms < lockout_until_ms) {
		return;
	}
	lockout_until_ms = now_ms + SELFTEST_LOCKOUT_MS;

	/* Normal (or another fault): one press = local fail, right away. */
	if (!z->self_test) {
		z->self_test = true;
		z->fault_to_send = CAN_FAULT_SELF_TEST;
		first_press_ms = -1;
		return;
	}

	/* Already in self-test: the second press within the window restores;
	 * otherwise this press becomes the first of a possible double press.
	 */
	if (first_press_ms >= 0 && now_ms - first_press_ms <= SELFTEST_DOUBLE_MS) {
		z->self_test = false;
		first_press_ms = -1;
	} else {
		first_press_ms = now_ms;
	}
}

void zone_update(struct zone *z, const struct zone_inputs *in, uint64_t now_us)
{
	on_test_button(z, in->test_btn, (int64_t)(now_us / 1000));

	/*
	 * TODO(cockpit): the state machine. Transitions (R3, S5, Cockpit Zone):
	 *
	 *   any        -> LOCAL_FAIL  z->self_test (set above; must happen
	 *                             within 10 ms of the press, R2.5)
	 *   any        -> LOCAL_FAIL  !udp_fresh (Cockpit Zone spec: no UDP for
	 *                             100 ms = cockpit failed, heartbeat stops);
	 *                             set fault_to_send = CAN_FAULT_UDP_LOST
	 *   LOCAL_FAIL -> ERROR       self_test cleared (double press) and the
	 *                             UDP stream is back
	 *   NORMAL     -> ERROR       peer_failed, isolated, or a FAULT arrived
	 *                             (zone_on_fault)
	 *   ERROR      -> NORMAL      peers_alive && udp_fresh && no peer_failed
	 *                             && not isolated && no recent FAULT
	 *                             (must happen within 500 ms of a restore)
	 *
	 * Lab 2 waited for 2 good commands in a row before clearing a fault
	 * (SYS_STATE_GOOD_TO_CLEAR) so one stray frame couldn't release the
	 * brakes. Consider the same for ERROR -> NORMAL (e.g. 2 heartbeats).
	 *
	 * Make sure a stale FAULT can't hold the system in ERROR forever: decide
	 * how long a received FAULT keeps you in ERROR after its zone recovers
	 * (e.g. until that zone's heartbeat is back).
	 */
	(void)in;
}

void zone_on_fault(struct zone *z, const struct can_fault *f, uint64_t now_us)
{
	/* TODO(cockpit): remember that zone f->zone reported f->reason at now_us,
	 * and go to ERROR from NORMAL (R3). Don't react to our own zone number.
	 */
	(void)z;
	(void)f;
	(void)now_us;
}
