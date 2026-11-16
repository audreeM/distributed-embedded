/* Global time on the Pi, see gtime.h. */
#include <pthread.h>
#include <stdio.h>

#include "gtime.h"
#include "timesync.h"
#include "util.h"

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static struct timesync ts;
static bool ready;

static void ensure_init(void)
{
	if (!ready) {
		ts_init(&ts, now_us());
		ready = true;
	}
}

uint64_t global_gettime(void)
{
	uint64_t now = now_us();

	pthread_mutex_lock(&lock);
	ensure_init();
	uint64_t g = ts_global(&ts, now);

	pthread_mutex_unlock(&lock);
	return g;
}

uint64_t gtime_global_to_local(uint64_t global_us)
{
	pthread_mutex_lock(&lock);
	ensure_init();
	uint64_t l = ts_local(&ts, global_us);

	pthread_mutex_unlock(&lock);
	return l;
}

void gtime_set_leader(bool leader)
{
	pthread_mutex_lock(&lock);
	ensure_init();
	ts_set_leader(&ts, leader);
	pthread_mutex_unlock(&lock);
}

void gtime_ref_received(int bus, uint8_t sender, const uint8_t d[8], uint64_t local_rx)
{
	pthread_mutex_lock(&lock);
	ensure_init();
	ts_ref_received(&ts, bus, sender, d, local_rx);
	pthread_mutex_unlock(&lock);
}

void gtime_ref_sent(int bus, uint8_t cycle, uint64_t local_done)
{
	pthread_mutex_lock(&lock);
	ensure_init();
	ts_ref_sent(&ts, bus, cycle, local_done);
	pthread_mutex_unlock(&lock);
}

void gtime_ref_fill(int bus, uint8_t d[8])
{
	pthread_mutex_lock(&lock);
	ensure_init();
	ts_ref_fill(&ts, bus, d);
	pthread_mutex_unlock(&lock);
}

bool gtime_synced(void)
{
	uint64_t now = now_us();

	pthread_mutex_lock(&lock);
	ensure_init();
	bool s = ts_is_synced(&ts, now);

	pthread_mutex_unlock(&lock);
	return s;
}

void gtime_get_stats(struct gtime_stats *s)
{
	uint64_t now = now_us();

	pthread_mutex_lock(&lock);
	ensure_init();
	s->leader = ts.leader;
	s->synced = ts_is_synced(&ts, now);
	s->drift_ppb = ts.drift_ppb;
	s->last_error_us = ts.last_error_us;
	s->samples = ts.samples;
	s->rejected = ts.rejected;
	pthread_mutex_unlock(&lock);
}

void gtime_fmt_drift(char *buf, int len, int64_t drift_ppb)
{
	int64_t a = drift_ppb < 0 ? -drift_ppb : drift_ppb;

	snprintf(buf, (size_t)len, "1%c%lld.%03lldppm", drift_ppb < 0 ? '-' : '+', (long long)(a / 1000),
		 (long long)(a % 1000));
}
