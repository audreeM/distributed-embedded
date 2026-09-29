/*
 * Zone fault state: ERROR while any LP_FAULT_* bit is set, NORMAL otherwise.
 *
 * Boots with LP_FAULT_POWERUP set, so the zone starts in the error state.
 * Everything that drives an actuator must check sys_state_is_error() and put
 * its output in the safe state (motor PWM off + dynamic brake, hazards) when
 * it returns true.
 */
#ifndef SYS_STATE_H
#define SYS_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "link_proto.h"

/* Consecutive good commands needed to clear POWERUP / LINK_LOST / BAD_CMD. */
#define SYS_STATE_GOOD_TO_CLEAR 2

/* Self-test button (wheel button carried as LP_BTN_TEST). */
#define SELFTEST_LOCKOUT_MS  50  /* leading-edge debounce */
#define SELFTEST_DOUBLE_MS   500 /* window for the restoring double press */

uint32_t sys_state_faults(void);
bool sys_state_is_error(void);

/* ISR-safe. */
void sys_state_set_fault(uint32_t bits);
void sys_state_clear_fault(uint32_t bits);

/* Called by the link RX thread only. */
void sys_state_on_good_cmd(void);
void sys_state_on_bad_cmd(void);
void sys_state_on_buttons(uint8_t buttons, int64_t now_ms);

#endif /* SYS_STATE_H */
