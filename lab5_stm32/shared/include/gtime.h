/*
 * Local clock and global time (part 6) on the STM32. Wraps the portable
 * common/timesync code with a lock and a microsecond clock.
 *
 * Local clock: the CPU cycle counter (84 MHz) stretched to 64 bits, in us.
 * The 10 kHz kernel tick is only 100 us, too coarse for time sync.
 */
#ifndef GTIME_H
#define GTIME_H

#include <stdbool.h>
#include <stdint.h>

/* Microseconds since boot, 1 us resolution. Safe from any thread. */
uint64_t gtime_local_us(void);

/* Part 6.3: the current global time in us, with the drift correction applied. */
uint64_t global_gettime(void);

/* Global time -> the local time it will happen at (for scheduling edges). */
uint64_t gtime_global_to_local(uint64_t global_us);

/* Sleep until local time `t`, waking EDGE_SPIN_US early and busy-waiting the rest. */
void gtime_wait_until(uint64_t local_us);

/* time sync hooks (see common/timesync/timesync.h) */
void gtime_set_leader(bool leader);
void gtime_ref_received(int bus, uint8_t sender, const uint8_t d[8], uint64_t local_rx);
void gtime_ref_sent(int bus, uint8_t cycle, uint64_t local_done);
void gtime_ref_fill(int bus, uint8_t d[8]);
bool gtime_synced(void);

struct gtime_stats {
	bool leader, synced;
	int64_t drift_ppb;     /* relative drift: (leader rate / our rate - 1) x 1e9 */
	int64_t last_error_us; /* correction applied at the last REF */
	uint32_t samples, rejected;
};
void gtime_get_stats(struct gtime_stats *s);

/* Prints "1+12.345ppm" style drift into buf. */
void gtime_fmt_drift(char *buf, int len, int64_t drift_ppb);

#endif /* GTIME_H */
