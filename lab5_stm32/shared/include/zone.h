/*
 * Zone state machine and heartbeat monitor (HB_MON), from lab 3. Lab 5 adds
 * the raft_obs it builds for the raft client, and reports this node's own
 * FAULTs as EVENT log entries.
 */
#ifndef ZONE_H
#define ZONE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_catalog.h"
#include "raft_client.h"

enum zone_state {
	ZONE_ERROR,
	ZONE_NORMAL,
	ZONE_LOCAL_FAIL,
};

enum zone_state zone_state(void);

/* our HB payload */
uint8_t zone_wire_state(void);
uint8_t zone_health_bits(void);

/* every received frame (duplicates too), for bus health */
void zone_on_frame(int bus, uint8_t sender, bool is_hb);
/* a heartbeat's payload (byte 2 = that node's degraded buses) */
void zone_on_hb(uint8_t sender, uint8_t health);
void zone_on_ctrl(enum can_check result);
void zone_on_fault(const struct can_fault *f);

/* is the cockpit around to send BLINK? */
bool zone_blink_master_alive(void);

/* what HB_MON sees, for the raft client and routing */
void zone_obs(struct raft_obs *o);

enum {
	WDT_HB_MON = BIT(0),
	WDT_HB_TX = BIT(1),
	WDT_RAFT = BIT(2),
	WDT_NODE_A = BIT(3),
	WDT_NODE_B = BIT(4),
	WDT_ALL = WDT_HB_MON | WDT_HB_TX | WDT_RAFT | WDT_NODE_A | WDT_NODE_B,
};
#if NODE_ID == 3
#define WDT_MOTOR WDT_NODE_A
#define WDT_PID   WDT_NODE_B
#else
#define WDT_STEER    WDT_NODE_A
#define WDT_FORCE_TX WDT_NODE_B
#endif
void zone_wdt_checkin(uint32_t bit);
uint32_t zone_wdt_take(void);

int zone_init(void);

#endif /* ZONE_H */
