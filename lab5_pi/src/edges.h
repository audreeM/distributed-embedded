/*
 * PPS and HZ outputs on the Pi, timed from global time by their own
 * realtime thread (the main loop only wakes every 1 ms, too coarse).
 *   PPS: rising edge at every whole second of global time (part 6.4)
 *   HZ:  the hazard lamps as the cockpit shows them (checkoff 1 scopes it
 *        next to FL and RL)
 */
#ifndef EDGES_H
#define EDGES_H

#include <stdbool.h>
#include <stdint.h>

/* The blink pattern the cockpit just announced in BLINK (main loop). */
void edges_set_pattern(uint8_t mode, uint64_t start_us, uint64_t half_us);

/* One PPS edge, for the main loop to print. */
struct pps_record {
	uint64_t global_us, local_us;
	int64_t drift_ppb, last_error_us;
	bool synced, leader;
};
/* Returns true and fills *r if an edge happened since the last call. */
bool edges_take_pps(struct pps_record *r);

int edges_start(void);
void edges_stop(void);

#endif /* EDGES_H */
