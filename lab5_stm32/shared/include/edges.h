/*
 * Timed outputs driven by global time: the blinker lamps and the PPS pin.
 * One high priority thread handles both so two edges at the same instant
 * (a whole second is also a hazard edge) don't delay each other.
 */
#ifndef EDGES_H
#define EDGES_H

#include <stdint.h>

#include "can_catalog.h"

/* A new, valid BLINK from the cockpit (dispatch thread). */
void blink_on_msg(const struct can_blink *b);

/* Re-check failure mode / own hazards now (zone state changed). */
void blink_kick(void);

/* One PPS edge, for main() to print (part 6.4). */
struct pps_record {
	uint64_t global_us; /* global_gettime() right at the edge */
	uint64_t local_us;  /* local clock at the same instant */
	int64_t drift_ppb;
	int64_t last_error_us;
	bool synced, leader;
};
extern struct k_msgq pps_q;

int edges_init(void);

#endif /* EDGES_H */
