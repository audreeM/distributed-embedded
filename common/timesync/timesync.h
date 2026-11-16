/*
 * Time sync for lab 5 (part 6). Portable C, shared by the Pi and both STM32s.
 *
 * Every node has a local clock (microseconds, from boot). Global time is
 * worked out from it with a straight line:
 *
 *   global(local) = base_global + (local - base_local) * (1 + drift)
 *
 * The leader's global time is the reference. It sends REF every 20 ms and
 * each REF carries the leader's global time when the previous REF finished
 * sending (two-step, see raft_msg.h). A follower timestamps each REF when it
 * arrives, so every REF after the first gives it one sample:
 *
 *   F = our local time when REF n-1 arrived
 *   L = leader's global time when REF n-1 finished sending
 *
 * Both ends timestamp the end of the same frame, which every node on a CAN
 * bus sees at the same moment, so the network delay cancels out (that's the
 * TTCAN trick from the handout). What's left is interrupt/driver latency.
 *
 * Each sample:
 *   - offset: move base_global toward L (half the error each time, which
 *     smooths out timestamp jitter)
 *   - drift: relative_drift = (L2 - L1) / (F2 - F1) from the handout. Over
 *     just 20 ms the timestamp jitter (tens of us) would swamp the drift (a
 *     few ppm), so instead of two samples we fit a line through the last
 *     TS_WINDOW samples (least squares, ~1.3 s) and use its slope.
 *
 * Between samples global() keeps applying the drift. A leader never takes
 * samples, its global time just keeps running with the last drift it had, so
 * the time stays continuous when leadership moves (checkoff 6: "no step
 * larger than 100 us at the handover").
 *
 * Not thread safe: the node wraps every call in its own lock.
 */
#ifndef TIMESYNC_H
#define TIMESYNC_H

#include <stdbool.h>
#include <stdint.h>

#define TS_WINDOW          128    /* samples used for drift (2 per REF, one per bus: ~1.3 s) */
#define TS_MIN_SPAN_US     500000 /* need samples spanning at least this long to estimate drift */
#define TS_FIT_EVERY       8      /* refit the drift every 8 samples (saves CPU on the STM32) */
#define TS_OUTLIER_US      500    /* error bigger than this = bad sample (e.g. a late relayed REF) */
#define TS_OUTLIER_ACCEPT  3      /* ...unless it happens this many times in a row: then it's real, jump */
#define TS_STEP_US         200    /* errors bigger than this are applied all at once, smaller ones halved */
#define TS_SYNC_LOST_US    200000 /* SYNC LED off after this long without a sample */
#define TS_MAX_DRIFT_PPB   1000000 /* 1000 ppm: crystals are +-30 ppm, anything past this is a bad fit */

struct ts_sample {
	uint64_t local;
	uint64_t global;
};

struct ts_rx { /* last REF received on a bus */
	bool valid;
	uint8_t sender;
	uint8_t cycle;
	uint64_t local;
};

struct ts_tx { /* last REF we finished sending on a bus */
	bool valid;
	uint8_t cycle;
	uint64_t global;
};

struct timesync {
	bool leader;
	bool synced; /* we've had at least one sample (or we're the leader) */

	/* the model */
	uint64_t base_local;
	uint64_t base_global;
	int64_t drift_ppb; /* parts per billion, + = our clock is slow vs the leader */

	/* stats for printing */
	uint64_t last_sample_local;
	int64_t last_error_us; /* how far off we were at the last sample (the correction applied) */
	uint32_t samples, rejected;

	uint8_t ref_sender; /* leader we're syncing to */
	int outliers;
	struct ts_sample win[TS_WINDOW];
	int win_n, win_head;

	struct ts_rx rx[2];
	struct ts_tx tx[2];
};

/* Starts unsynced with global = local. */
void ts_init(struct timesync *ts, uint64_t local_now);

/* local -> global and back */
uint64_t ts_global(const struct timesync *ts, uint64_t local);
uint64_t ts_local(const struct timesync *ts, uint64_t global);

/* Leader keeps its time running and stops listening to REFs. */
void ts_set_leader(struct timesync *ts, bool leader);

/* A REF frame arrived on `bus` from `sender` at local time `local_rx`. Call for every copy. */
void ts_ref_received(struct timesync *ts, int bus, uint8_t sender, const uint8_t d[8], uint64_t local_rx);

/* A REF we sent (as leader or relay) finished sending on `bus` at local time `local_done`. */
void ts_ref_sent(struct timesync *ts, int bus, uint8_t cycle, uint64_t local_done);

/* Fill in bytes 3-7 of a REF we're about to send on `bus` (d[2] = its cycle). */
void ts_ref_fill(const struct timesync *ts, int bus, uint8_t d[8]);

/* Leader, or had a sample recently. Drives the SYNC LED. */
bool ts_is_synced(const struct timesync *ts, uint64_t local_now);

#endif /* TIMESYNC_H */
