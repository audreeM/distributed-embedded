/* Time sync, see timesync.h. */
#include <string.h>

#include "timesync.h"
#include "../raft/raft_msg.h"

#define BILLION 1000000000LL

void ts_init(struct timesync *ts, uint64_t local_now)
{
	memset(ts, 0, sizeof(*ts));
	ts->base_local = local_now;
	ts->base_global = local_now;
}

uint64_t ts_global(const struct timesync *ts, uint64_t local)
{
	int64_t dt = (int64_t)(local - ts->base_local);

	return ts->base_global + (uint64_t)(dt + dt * ts->drift_ppb / BILLION);
}

uint64_t ts_local(const struct timesync *ts, uint64_t global)
{
	int64_t dg = (int64_t)(global - ts->base_global);

	/* inverse of the line above; drift^2 is way below a microsecond */
	return ts->base_local + (uint64_t)(dg - dg * ts->drift_ppb / BILLION);
}

void ts_set_leader(struct timesync *ts, bool leader)
{
	if (leader == ts->leader) {
		return;
	}
	ts->leader = leader;
	if (leader) {
		ts->synced = true; /* our clock is the reference now */
	}
	/* start over on pairing, the REFs we'll get next are from someone else */
	ts->rx[0].valid = ts->rx[1].valid = false;
	ts->win_n = 0;
	ts->outliers = 0;
}

/* A REF only carries 40 bits of time: put back the high bits, picking the
 * value closest to what we think global time is. */
static uint64_t unwrap40(uint64_t t40, uint64_t estimate)
{
	uint64_t full = (estimate & ~RAFT_TIME_MASK) | t40;
	const uint64_t span = RAFT_TIME_MASK + 1;

	if (full > estimate && full - estimate > span / 2 && full >= span) {
		full -= span;
	} else if (full < estimate && estimate - full > span / 2) {
		full += span;
	}
	return full;
}

static void window_push(struct timesync *ts, uint64_t local, uint64_t global)
{
	ts->win[ts->win_head] = (struct ts_sample){local, global};
	ts->win_head = (ts->win_head + 1) % TS_WINDOW;
	if (ts->win_n < TS_WINDOW) {
		ts->win_n++;
	}
}

/*
 * Slope of the best line through the window, minus 1, in ppb. That's
 * (L2 - L1) / (F2 - F1) - 1 from the handout, averaged over every pair
 * (least squares). Integer math only: the STM32's FPU has no doubles.
 * Returns false if the samples don't span enough time yet.
 */
static bool window_fit(const struct timesync *ts, int64_t *drift_ppb)
{
	int first = (ts->win_head - ts->win_n + TS_WINDOW) % TS_WINDOW;
	const struct ts_sample *s0 = &ts->win[first];
	int64_t sx = 0, sy = 0, sxx = 0, sxy = 0, span = 0, mx, my;

	if (ts->win_n < 2) {
		return false;
	}
	/* x = local time since the first sample, y = how much further global moved (drift + noise) */
	for (int k = 0; k < ts->win_n; k++) {
		const struct ts_sample *s = &ts->win[(first + k) % TS_WINDOW];
		int64_t x = (int64_t)(s->local - s0->local);

		sx += x;
		sy += (int64_t)(s->global - s0->global) - x;
		if (x > span) {
			span = x;
		}
	}
	if (span < TS_MIN_SPAN_US) {
		return false;
	}
	mx = sx / ts->win_n;
	my = sy / ts->win_n;
	for (int k = 0; k < ts->win_n; k++) {
		const struct ts_sample *s = &ts->win[(first + k) % TS_WINDOW];
		int64_t x = (int64_t)(s->local - s0->local);
		int64_t dx = x - mx;
		int64_t dy = (int64_t)(s->global - s0->global) - x - my;

		sxx += dx * dx;
		sxy += dx * dy;
	}
	if (sxx < 1000000) {
		return false;
	}
	/* sxy / sxx * 1e9, split so nothing overflows 64 bits */
	*drift_ppb = sxy * 1000 / (sxx / 1000000);
	return true;
}

/* One (F, L) pair: our local time and the leader's global time for the same instant. */
static void take_sample(struct timesync *ts, uint8_t sender, uint64_t f, uint64_t l40)
{
	uint64_t predicted = ts_global(ts, f);
	uint64_t l = unwrap40(l40, predicted);
	int64_t err = (int64_t)(l - predicted);

	if (sender != ts->ref_sender) {
		/* new leader: its clock should match the old one, but start the
		 * drift window over since it's a different crystal */
		ts->ref_sender = sender;
		ts->win_n = 0;
		ts->outliers = 0;
	}

	if (!ts->synced) {
		/* first sample ever: just take the leader's time */
		ts->base_local = f;
		ts->base_global = l;
		ts->synced = true;
		ts->last_error_us = err;
		ts->last_sample_local = f;
		ts->samples++;
		window_push(ts, f, l);
		return;
	}

	if ((err > TS_OUTLIER_US || err < -TS_OUTLIER_US) && ++ts->outliers < TS_OUTLIER_ACCEPT) {
		ts->rejected++;
		return;
	}
	ts->outliers = 0;

	/* a jump means the old samples were measured against a different
	 * time (e.g. a relay that wasn't synced yet), so don't fit across it */
	if (err > TS_STEP_US || err < -TS_STEP_US) {
		ts->win_n = 0;
	}
	window_push(ts, f, l);
	if (ts->samples % TS_FIT_EVERY == 0) {
		int64_t d;

		if (window_fit(ts, &d) && d < TS_MAX_DRIFT_PPB && d > -TS_MAX_DRIFT_PPB) {
			ts->drift_ppb = d;
		}
	}

	/* move the line to this sample: all the way for big errors, halfway for small ones */
	ts->base_local = f;
	ts->base_global = (err > TS_STEP_US || err < -TS_STEP_US) ? l : predicted + (uint64_t)(err / 2);
	ts->last_error_us = err;
	ts->last_sample_local = f;
	ts->samples++;
}

void ts_ref_received(struct timesync *ts, int bus, uint8_t sender, const uint8_t d[8], uint64_t local_rx)
{
	struct ts_rx prev = ts->rx[bus];
	uint8_t cycle = d[2];
	uint64_t t = raft_get40(&d[3]);

	if (ts->leader) {
		return;
	}
	ts->rx[bus] = (struct ts_rx){.valid = true, .sender = sender, .cycle = cycle, .local = local_rx};

	/* this REF carries the time of the previous one: pair it with when we got that one */
	if (!prev.valid || prev.sender != sender || (uint8_t)(prev.cycle + 1) != cycle ||
	    t == RAFT_TIME_INVALID) {
		return;
	}
	take_sample(ts, sender, prev.local, t);
}

void ts_ref_sent(struct timesync *ts, int bus, uint8_t cycle, uint64_t local_done)
{
	/* a relay that isn't synced yet has nothing useful to pass on */
	ts->tx[bus] = (struct ts_tx){
		.valid = ts->leader || ts->synced, .cycle = cycle, .global = ts_global(ts, local_done)};
}

void ts_ref_fill(const struct timesync *ts, int bus, uint8_t d[8])
{
	const struct ts_tx *tx = &ts->tx[bus];
	uint64_t t = RAFT_TIME_INVALID;

	if (tx->valid && (uint8_t)(tx->cycle + 1) == d[2]) {
		t = tx->global & RAFT_TIME_MASK;
		if (t == RAFT_TIME_INVALID) {
			t--; /* 1 us off once every 12 days, so it never looks like "no time" */
		}
	}
	raft_put40(&d[3], t);
}

bool ts_is_synced(const struct timesync *ts, uint64_t local_now)
{
	return ts->leader || (ts->synced && local_now - ts->last_sample_local < TS_SYNC_LOST_US);
}
