/*
 * The cockpit's two SocketCAN interfaces: bus A (can0) and bus B (can1).
 * Lab 5: frames come with the kernel's receive timestamp (time sync), and
 * our own frames come back to us when they finish sending (so the Pi knows
 * when its REF actually went out on the wire).
 */
#ifndef CANBUS_H
#define CANBUS_H

#include <stdbool.h>
#include <stdint.h>

#include "can_catalog.h"

enum { BUS_A = 0, BUS_B = 1, BUS_COUNT = 2 };

struct canbus {
	int fd[BUS_COUNT];          /* non-blocking raw sockets, -1 if not open */
	uint32_t tx_ok[BUS_COUNT]; 	/* frames the kernel accepted */
	uint32_t tx_fail[BUS_COUNT]; /* writes refused: queue full (bus not ACKing) */
	uint32_t rx[BUS_COUNT];     /* frames received, duplicates included */
};

/* Opens both interfaces. Returns 0 if at least one opened. */
int canbus_open(struct canbus *cb);

/*
 * Queue one frame on bus A, then bus B, back to back, toggling CAN_TX_A /
 * CAN_TX_B right before each write. Never blocks: a bus nobody ACKs just
 * fails its write. Returns a bitmask of the buses that accepted it.
 */
unsigned canbus_send(struct canbus *cb, uint16_t id, const uint8_t data[CAN_DLC]);

/* One frame on one bus only (relay, raft REF). Returns 0 if the kernel took it. */
int canbus_send_on(struct canbus *cb, int bus, uint16_t id, const uint8_t data[CAN_DLC]);

/*
 * Read one frame from one bus. Returns 1 with *id, data and *dlc filled,
 * 0 if nothing is waiting, -1 on error. *t_us = when the kernel got it
 * (CLOCK_MONOTONIC us). *own = this is our own frame coming back after it
 * finished sending.
 */
int canbus_recv(struct canbus *cb, int bus, uint16_t *id, uint8_t data[CAN_DLC], uint8_t *dlc,
		uint64_t *t_us, bool *own);

void canbus_close(struct canbus *cb);

#endif /* CANBUS_H */
