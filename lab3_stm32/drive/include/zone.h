#ifndef ZONE_H
#define ZONE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_catalog.h"

enum zone_state {
	ZONE_ERROR,      
	ZONE_NORMAL,
	ZONE_LOCAL_FAIL, 
};

enum zone_state zone_state(void);

/* HB_N2 payload. */
uint8_t zone_wire_state(void);
uint8_t zone_health_bits(void);

void zone_on_frame(int bus, uint8_t sender, bool is_hb);
void zone_on_ctrl(enum can_check result);
void zone_on_fault(const struct can_fault *f);

bool zone_relay_to(uint8_t sender, int from_bus, int *to_bus);


bool zone_blink_master_alive(void);

enum {
	WDT_HB_MON = BIT(0),
	WDT_HB_TX = BIT(1),
	WDT_MOTOR = BIT(2),
	WDT_PID = BIT(3),
	WDT_ALL = WDT_HB_MON | WDT_HB_TX | WDT_MOTOR | WDT_PID,
};
void zone_wdt_checkin(uint32_t bit);
uint32_t zone_wdt_take(void);

int zone_init(void);

#endif