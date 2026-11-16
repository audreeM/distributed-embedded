/*
 * Global time on the Pi (part 6). Wraps common/timesync with a mutex, since
 * the main loop and the edges thread both use it. Local clock = CLOCK_MONOTONIC
 * in us (util.h now_us()).
 */
#ifndef GTIME_H
#define GTIME_H

#include <stdbool.h>
#include <stdint.h>

/* Part 6.3: the current global time in us, with the drift correction applied. */
uint64_t global_gettime(void);

/* Global time -> the local (CLOCK_MONOTONIC us) time it will happen at. */
uint64_t gtime_global_to_local(uint64_t global_us);

/* time sync hooks (see common/timesync/timesync.h) */
void gtime_set_leader(bool leader);
void gtime_ref_received(int bus, uint8_t sender, const uint8_t d[8], uint64_t local_rx);
void gtime_ref_sent(int bus, uint8_t cycle, uint64_t local_done);
void gtime_ref_fill(int bus, uint8_t d[8]);
bool gtime_synced(void);

struct gtime_stats {
	bool leader, synced;
	int64_t drift_ppb;
	int64_t last_error_us;
	uint32_t samples, rejected;
};
void gtime_get_stats(struct gtime_stats *s);

/* "1+12.345ppm" */
void gtime_fmt_drift(char *buf, int len, int64_t drift_ppb);

#endif /* GTIME_H */
