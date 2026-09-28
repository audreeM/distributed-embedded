/**
 * @file state.c
 * @brief state (normal, error, local fail) logic
 */

#include <zephyr/kernel.h>
#include "state.h"

static zone_state_t state = ST_ERROR;
static uint8_t reason = R_POWER_UP;

// guards state + reason. k_spin_lock disables interrupts on this single-core
// part, so the link watchdog ISR can't run between a check and a write.
// keep critical sections to a few writes: no printk, no sleeping
static struct k_spinlock lock;

// on startup the system is in error state (brakes engaged and hazard lights)
void state_init(void) {
	k_spinlock_key_t key = k_spin_lock(&lock);

	state = ST_ERROR;
	reason = R_POWER_UP;

	// TODO: add brakes and hazard light functions

	k_spin_unlock(&lock, key);
}

// puts the zone in error state. safe to call from an ISR (link watchdog)
void enter_error(uint8_t reasonUpdated) {
	// check if it is in local_fail??
	k_spinlock_key_t key = k_spin_lock(&lock);

	state = ST_ERROR;
	reason = reasonUpdated;
	// TODO: add brake and hazard light functions

	k_spin_unlock(&lock, key);
}

// puts the zone in normal state
void enter_normal(void) {
	k_spinlock_key_t key = k_spin_lock(&lock);

	state = ST_NORMAL;

	k_spin_unlock(&lock, key);
}