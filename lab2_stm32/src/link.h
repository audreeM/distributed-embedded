/*
 * Pi -> STM32 command link and STM32 -> Pi status heartbeat (USART1).
 *
 * Usage from the rest of the app:
 *   - link_get_cmd()      latest accepted command, any thread, never blocks
 *   - link_wait()         block until a new command OR a fault-state change;
 *                         meant for ONE consumer (the drivetrain control
 *                         thread) so brake / fail-safe are acted on at once
 *   - link_set_currents() current-sensor readings reported in the status frame
 */
#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#include "link_proto.h"

#define LINK_TIMEOUT_MS   70 /* 3 missed 20 ms command refreshes + margin */
#define LINK_STATUS_MS    20 /* status frame period */
#define LINK_RX_PRIO      1  /* link RX thread; lower number = higher priority */

struct link_cmd {
	bool valid;     /* false until the first accepted command */
	uint8_t seq;
	uint8_t throttle; /* 0..100 % */
	uint8_t brake;    /* 0..100 % */
	int8_t steer;     /* -100..+100 */
	uint8_t buttons;  /* LP_BTN_* */
	int64_t rx_ms;    /* k_uptime_get() at acceptance */
};

int link_init(void);
void link_get_cmd(struct link_cmd *out);
bool link_wait(k_timeout_t timeout);
void link_set_currents(uint16_t motor_l_ma, uint16_t motor_r_ma, uint16_t servo_ma);

/* Wakes link_wait(); ISR-safe. Used by sys_state on fault changes. */
void link_notify(void);

#endif /* LINK_H */
