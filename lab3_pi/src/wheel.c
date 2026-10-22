/*
 * Raw DIJOYSTATE2 -> CAN units. Same scaling as Lab 2: the conversion happens
 * on the Pi so the STM32 zones never need to know which proxy or wheel
 * calibration is upstream.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "state.h"
#include "wheel.h"

/* Fail the build (instead of silently misparsing) if the struct changes size. */
_Static_assert(sizeof(DIJOYSTATE2_t) == WHEEL_PKT_LEN - 4, "DIJOYSTATE2 layout");

/* Linear map raw [from..to] -> [lo..hi], clamped. Works for reversed ranges. */
static int map_axis(long raw, long from, long to, int lo, int hi)
{
	if (to == from) {
		return lo;
	}
	double t = (double)(raw - from) / (double)(to - from);

	if (t < 0.0) {
		t = 0.0;
	} else if (t > 1.0) {
		t = 1.0;
	}
	return (int)lround(lo + t * (hi - lo));
}

/* DirectInput marks a pressed button with the high bit of its byte. */
static bool button(const DIJOYSTATE2_t *s, int idx)
{
	return idx >= 0 && idx < 128 && (s->rgbButtons[idx] & 0x80);
}

bool wheel_parse(const uint8_t *buf, size_t len, struct wheel *w)
{
	DIJOYSTATE2_t s;

	if (len != WHEEL_PKT_LEN) {
		return false;
	}
	memcpy(&s, buf + 4, sizeof(s)); /* skip the 4-byte counter; buf+4 is not int-aligned */

	/* Each half of the steering range is mapped separately, so a center
	 * that is not exactly halfway still gives 0 at center. Monotonic.
	 */
	int steer = (s.lX < STEER_RAW_CENTER)
			    ? map_axis(s.lX, STEER_RAW_LEFT, STEER_RAW_CENTER, -100, 0)
			    : map_axis(s.lX, STEER_RAW_CENTER, STEER_RAW_RIGHT, 0, 100);

	if (abs(steer) <= STEER_DEADBAND) {
		steer = 0;
	}
	int thr = map_axis(s.lY, THR_RAW_RELEASED, THR_RAW_PRESSED, 0, 100);
	int brk = map_axis(s.lRz, BRK_RAW_RELEASED, BRK_RAW_PRESSED, 0, 100);

	w->ctrl.steer = (int16_t)steer;
	w->ctrl.throttle = (uint8_t)(thr < THR_DEADBAND ? 0 : thr);
	w->ctrl.brake = (uint8_t)(brk < BRK_DEADBAND ? 0 : brk);

	w->left = button(&s, BTN_IDX_LEFT);
	w->right = button(&s, BTN_IDX_RIGHT);
	w->test = button(&s, BTN_IDX_TEST);

	/* Both turn buttons at once is invalid in CTRL; report neither. */
	w->ctrl.buttons = 0;
	if (w->left != w->right) {
		w->ctrl.buttons |= w->left ? CAN_BTN_LEFT : CAN_BTN_RIGHT;
	}
	if (w->test) {
		w->ctrl.buttons |= CAN_BTN_TEST;
	}
	return true;
}
