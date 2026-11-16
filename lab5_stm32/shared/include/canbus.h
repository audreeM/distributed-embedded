/*
 * The two MCP2515 modules: bus A (can_a) and bus B (can_b). Same as lab 3
 * plus: every received frame gets a local timestamp (for time sync), and REF
 * frames get a "finished sending" timestamp.
 */
#ifndef CANBUS_H
#define CANBUS_H

#include <stdint.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include "can_catalog.h"

enum { BUS_A = 0, BUS_B = 1, BUS_COUNT = 2 };

/* One received frame, the bus it came in on, and when (gtime_local_us). */
struct rx_item {
	struct can_frame frame;
	uint64_t t_us;
	uint8_t bus;
};

/* Every received frame from both buses, duplicates included (dispatch.c). */
extern struct k_msgq canbus_rxq;

/* Sets up the receive filters on both modules and starts them. 0 if at least one came up. */
int canbus_init(void);

/*
 * Lab 3's send helper: the same 8 bytes on bus A, then bus B, then bump *seq.
 * Never blocks for long: on a degraded bus it doesn't wait for the TX buffer.
 */
void canbus_send_both(uint16_t id, const uint8_t d[CAN_DLC], uint8_t *seq);

/* One frame on one bus only, unchanged (relay, raft). Returns 0 if queued. */
int canbus_send_on(int bus, uint16_t id, const uint8_t d[CAN_DLC]);

/*
 * A REF frame on one bus. Fills in the time from the last REF we sent on that
 * bus (time sync) and timestamps this one when it finishes sending.
 */
int canbus_send_ref_on(int bus, uint16_t id, const uint8_t d[CAN_DLC]);

/* Counters for the status line. */
struct canbus_stats {
	uint32_t tx_ok[BUS_COUNT];
	uint32_t tx_fail[BUS_COUNT];
	uint32_t rx[BUS_COUNT];
	uint32_t rx_drop;
	enum can_state state[BUS_COUNT];
};
void canbus_get_stats(struct canbus_stats *s);

#endif /* CANBUS_H */
