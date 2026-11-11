/*
 * N3's two MCP2515 modules: bus A (can_a) and bus B (can_b).
 * Same job as lab3_pi/src/canbus.c, on Zephyr.
 */
#ifndef CANBUS_H
#define CANBUS_H

#include <stdint.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include "can_catalog.h"

enum { BUS_A = 0, BUS_B = 1, BUS_COUNT = 2 };

/* One received frame and the bus it came in on. */
struct rx_item {
	struct can_frame frame;
	uint8_t bus;
};

/* Every received frame from both buses, duplicates included (dispatch.c). */
extern struct k_msgq canbus_rxq;

/* Registers the catalog IDs on both modules and starts them. 0 if at least one came up. */
int canbus_init(void);

/*
 * The send helper (Part 5): queue the same 8 bytes on bus A, then bus B, back
 * to back, toggling CAN_TX_A / CAN_TX_B, then bump *seq once. The caller packs
 * d with the current *seq. Never blocks: a bus nobody ACKs just fails its send.
 */
void canbus_send_both(uint16_t id, const uint8_t d[CAN_DLC], uint8_t *seq);

/* Queue one frame on one bus only, unchanged (the relay). Returns 0 if queued. */
int canbus_send_on(int bus, uint16_t id, const uint8_t d[CAN_DLC]);

/* Counters for the status line. */
struct canbus_stats {
	uint32_t tx_ok[BUS_COUNT];
	uint32_t tx_fail[BUS_COUNT]; /* not queued (TX buffers full / bus-off) or TX error */
	uint32_t rx[BUS_COUNT];
	uint32_t rx_drop;            /* canbus_rxq was full */
	enum can_state state[BUS_COUNT];
};
void canbus_get_stats(struct canbus_stats *s);

#endif /* CANBUS_H */