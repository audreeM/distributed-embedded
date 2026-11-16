/* Test points and LEDs. Any pin missing from the board overlay is skipped. */
#ifndef TP_H
#define TP_H

#include <stdbool.h>

enum tp {
	TP_CAN_RX_A,
	TP_CAN_RX_B,
	TP_CAN_TX_A,
	TP_CAN_TX_B,
	TP_DEG_A,
	TP_DEG_B,
	TP_RELAY_TX,
	TP_ERR,
	/* lab 5 */
	TP_LEADER, /* LED: this node is the raft leader */
	TP_SYNC,   /* LED: global time is synced (leader, or REF heard recently) */
	TP_REF_RX, /* toggles on every new REF received */
	TP_PPS,    /* rising edge at every whole second of global time */
	TP_COUNT,
};

int tp_init(void);
void tp_toggle(enum tp tp);
void tp_set(enum tp tp, bool on);

#endif /* TP_H */
