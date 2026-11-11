/* Time helpers shared by cockpit module. All timestamps are microseconds. */
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <time.h>

#define MS(x) ((uint64_t)(x) * 1000u)

/* Monotonic clock: unaffected by NTP / wall-clock changes. */
static inline uint64_t now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* Milliseconds for can_seq_accept(); wraps after 49 days, which it handles. */
static inline uint32_t us_to_ms32(uint64_t us)
{
	return (uint32_t)(us / 1000u);
}

#endif /* UTIL_H */
