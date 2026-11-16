/* Local clock + global time on the STM32 (part 6). See gtime.h. */
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/clock.h>

#include "config.h"
#include "gtime.h"
#include "timesync.h"

static struct k_spinlock clk_lock;
static uint32_t last_cyc;
static uint64_t cyc_high;

uint64_t gtime_local_us(void)
{
	/* k_cycle_get_32() wraps every 51 s at 84 MHz. Count the wraps; this
	 * gets called many times a second, so we never miss one. */
	k_spinlock_key_t key = k_spin_lock(&clk_lock);
	uint32_t c = k_cycle_get_32();

	if (c < last_cyc) {
		cyc_high += 1ull << 32;
	}
	last_cyc = c;
	uint64_t cyc = cyc_high | c;

	k_spin_unlock(&clk_lock, key);
	return k_cyc_to_us_floor64(cyc);
}

/* all timesync calls go through this lock (dispatch, CAN driver, edges, raft threads) */
static struct k_spinlock ts_lock;
static struct timesync ts;
static bool ts_ready;

static void ensure_init(void)
{
	if (!ts_ready) {
		ts_init(&ts, gtime_local_us());
		ts_ready = true;
	}
}

uint64_t global_gettime(void)
{
	uint64_t now = gtime_local_us();
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	uint64_t g = ts_global(&ts, now);

	k_spin_unlock(&ts_lock, key);
	return g;
}

uint64_t gtime_global_to_local(uint64_t global_us)
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	uint64_t l = ts_local(&ts, global_us);

	k_spin_unlock(&ts_lock, key);
	return l;
}

void gtime_wait_until(uint64_t local_us)
{
	uint64_t now = gtime_local_us();

	if (local_us > now + EDGE_SPIN_US) {
		k_sleep(K_TIMEOUT_ABS_US(local_us - EDGE_SPIN_US));
	}
	while (gtime_local_us() < local_us) {
		/* spin the last few hundred us: the kernel tick is 100 us */
	}
}

void gtime_set_leader(bool leader)
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	ts_set_leader(&ts, leader);
	k_spin_unlock(&ts_lock, key);
}

/* Part 6.1: also set the system clock (SYS_CLOCK_REALTIME) to global time, like
 * clock_settime(). Only to the kernel tick (100 us); global_gettime() is the precise one. */
static void set_system_time(uint64_t global_us)
{
	struct timespec tsp = {
		.tv_sec = (time_t)(global_us / 1000000u),
		.tv_nsec = (long)(global_us % 1000000u) * 1000,
	};

	(void)sys_clock_settime(SYS_CLOCK_REALTIME, &tsp);
}

void gtime_ref_received(int bus, uint8_t sender, const uint8_t d[8], uint64_t local_rx)
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	uint32_t before = ts.samples;

	ts_ref_received(&ts, bus, sender, d, local_rx);
	bool took = ts.samples != before;
	uint64_t g = ts_global(&ts, gtime_local_us());

	k_spin_unlock(&ts_lock, key);
	if (took) {
		set_system_time(g);
	}
}

void gtime_ref_sent(int bus, uint8_t cycle, uint64_t local_done)
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	ts_ref_sent(&ts, bus, cycle, local_done);
	k_spin_unlock(&ts_lock, key);
}

void gtime_ref_fill(int bus, uint8_t d[8])
{
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	ts_ref_fill(&ts, bus, d);
	k_spin_unlock(&ts_lock, key);
}

bool gtime_synced(void)
{
	uint64_t now = gtime_local_us();
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	bool s = ts_is_synced(&ts, now);

	k_spin_unlock(&ts_lock, key);
	return s;
}

void gtime_get_stats(struct gtime_stats *s)
{
	uint64_t now = gtime_local_us();
	k_spinlock_key_t key = k_spin_lock(&ts_lock);

	ensure_init();
	s->leader = ts.leader;
	s->synced = ts_is_synced(&ts, now);
	s->drift_ppb = ts.drift_ppb;
	s->last_error_us = ts.last_error_us;
	s->samples = ts.samples;
	s->rejected = ts.rejected;
	k_spin_unlock(&ts_lock, key);
}

void gtime_fmt_drift(char *buf, int len, int64_t drift_ppb)
{
	/* ratio = 1 + drift, shown in ppm with 3 decimals (no float printf on this build) */
	int64_t a = drift_ppb < 0 ? -drift_ppb : drift_ppb;

	snprintk(buf, len, "1%c%lld.%03lldppm", drift_ppb < 0 ? '-' : '+', (long long)(a / 1000),
		 (long long)(a % 1000));
}
