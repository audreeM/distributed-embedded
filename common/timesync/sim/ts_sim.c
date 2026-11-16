/*
 * Tests timesync.c on a laptop. Three nodes with different crystal drift and
 * random timestamp latency, REF every 20 ms, like on the real buses.
 *
 *   make && ./ts_sim        all tests
 *   ./ts_sim -v             also print one line per simulated second
 *
 * Checks:
 *  1. followers converge to the leader (error at each PPS edge)
 *  2. leader dies, a follower takes over: no step in anyone's global time
 *  3. a follower only reachable through a relay (relay re-stamps REF on its bus)
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "timesync.h"
#include "raft_msg.h"

#define N 3

static int verbose;
static uint32_t rng = 12345;

static uint32_t rnd(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return rng;
}

/* random latency in [lo, hi] us */
static double lat(double lo, double hi) { return lo + (hi - lo) * (rnd() % 10000) / 10000.0; }

struct node {
	double ppm;    /* crystal error */
	double offset; /* local clock at true time 0 */
	struct timesync ts;
	bool alive;
};

static struct node nodes[N + 1];

/* local clock of node i at true time t (us) */
static uint64_t local_at(int i, double t)
{
	return (uint64_t)(nodes[i].offset + t * (1.0 + nodes[i].ppm * 1e-6));
}

static uint8_t cycle;

/*
 * Leader sends one REF on both buses at true time t. Receivers on that bus get
 * it at the end of the frame (same instant for everyone) plus their own
 * receive latency; the leader gets its "sent" callback at the same instant plus
 * its own latency. relay >= 0: that node copies the bus A frame to bus B.
 */
static void send_ref(int leader, double t, bool on_bus[N + 1][2], int relay)
{
	uint8_t d[8] = {0};
	double frame_end = t + 250; /* one 8-byte frame at 500 kbit/s */

	d[2] = cycle;
	for (int bus = 0; bus < 2; bus++) {
		if (!on_bus[leader][bus]) {
			continue;
		}
		ts_ref_fill(&nodes[leader].ts, bus, d);
		for (int j = 1; j <= N; j++) {
			if (j != leader && nodes[j].alive && on_bus[j][bus]) {
				ts_ref_received(&nodes[j].ts, bus, (uint8_t)leader, d,
						local_at(j, frame_end + lat(20, 80)));
			}
		}
		ts_ref_sent(&nodes[leader].ts, bus, cycle, local_at(leader, frame_end + lat(20, 80)));
	}
	if (relay > 0) {
		/* relay got the bus A copy, sends it on B ~400 us later with its own time in it */
		double relay_end = frame_end + lat(300, 600) + 250;

		ts_ref_fill(&nodes[relay].ts, 1, d);
		for (int j = 1; j <= N; j++) {
			if (j != leader && j != relay && nodes[j].alive && on_bus[j][1] && !on_bus[j][0]) {
				ts_ref_received(&nodes[j].ts, 1, (uint8_t)leader, d,
						local_at(j, relay_end + lat(20, 80)));
			}
		}
		ts_ref_sent(&nodes[relay].ts, 1, cycle, local_at(relay, relay_end + lat(20, 80)));
	}
	cycle++;
}

/* global time each live node reports at true time t, as an offset from node `ref` */
static double worst_spread(double t, int *who)
{
	double lo = 1e18, hi = -1e18;

	for (int i = 1; i <= N; i++) {
		if (!nodes[i].alive) {
			continue;
		}
		double g = (double)ts_global(&nodes[i].ts, local_at(i, t));

		if (g < lo) {
			lo = g;
		}
		if (g > hi) {
			hi = g;
		}
	}
	(void)who;
	return hi - lo;
}

static void setup(void)
{
	double ppm[N + 1] = {0, 12.0, -21.0, 27.0};  /* within the +-30 ppm crystals */
	double off[N + 1] = {0, 35e6, 1.2e6, 0.9e6}; /* Pi booted way earlier */

	for (int i = 1; i <= N; i++) {
		nodes[i] = (struct node){.ppm = ppm[i], .offset = off[i], .alive = true};
		ts_init(&nodes[i].ts, local_at(i, 0));
	}
	cycle = 0;
}

static int check(const char *what, double got, double limit)
{
	int ok = got <= limit;

	printf("  %-52s %8.1f us (limit %.0f)  %s\n", what, got, limit, ok ? "ok" : "FAIL");
	return ok;
}

/* runs REFs from `leader` from t0 to t1 (us), returns worst spread over the last half */
static double run(int leader, double t0, double t1, bool on_bus[N + 1][2], int relay)
{
	double worst = 0;

	for (double t = t0; t < t1; t += 20000) {
		send_ref(leader, t, on_bus, relay);
		if (t > (t0 + t1) / 2) {
			double s = worst_spread(t + 10000, NULL);

			if (s > worst) {
				worst = s;
			}
		}
		if (verbose && (int64_t)t % 1000000 == 0) {
			printf("    t=%5.1fs", t / 1e6);
			for (int i = 1; i <= N; i++) {
				if (nodes[i].alive) {
					printf("  N%d err %+5" PRId64 "us drift %+7.3fppm", i,
					       nodes[i].ts.last_error_us, nodes[i].ts.drift_ppb / 1000.0);
				}
			}
			printf("\n");
		}
	}
	return worst;
}

int main(int argc, char **argv)
{
	bool all_on[N + 1][2] = {{0}, {1, 1}, {1, 1}, {1, 1}};
	bool hop[N + 1][2] = {{0}, {1, 0}, {1, 1}, {0, 1}}; /* Pi only on A, N3 only on B; N2 relays */
	int ok = 1;

	verbose = argc > 1 && strcmp(argv[1], "-v") == 0;

	printf("1. Pi leads, both STM32s follow\n");
	setup();
	ts_set_leader(&nodes[1].ts, true);
	ok &= check("spread between nodes after it settles", run(1, 0, 10e6, all_on, 0), 50);
	printf("     N2 drift estimate %+.2f ppm (true %+.2f)\n", nodes[2].ts.drift_ppb / 1000.0,
	       (1 + nodes[1].ppm * 1e-6) / (1 + nodes[2].ppm * 1e-6) * 1e6 - 1e6);
	ok &= check("N2 drift estimate error (ppm x 1000)",
		    abs((int)(nodes[2].ts.drift_ppb -
			      ((1 + nodes[1].ppm * 1e-6) / (1 + nodes[2].ppm * 1e-6) - 1) * 1e9)) / 1.0,
		    8000);

	printf("2. Pi dies, N2 takes over (no step at the handover)\n");
	{
		double t = 10e6;
		double g_before = (double)ts_global(&nodes[3].ts, local_at(3, t + 200000));

		nodes[1].alive = false;
		ts_set_leader(&nodes[2].ts, true);
		/* 200 ms with no REF while the election happens */
		double g_n2 = (double)ts_global(&nodes[2].ts, local_at(2, t + 200000));

		ok &= check("N2 vs N3 global time when N2 starts leading", g_n2 > g_before ? g_n2 - g_before : g_before - g_n2, 100);
		run(2, t + 200000, t + 200000 + 40000, all_on, 0);
		double step = (double)ts_global(&nodes[3].ts, local_at(3, t + 260000)) -
			      (double)ts_global(&nodes[2].ts, local_at(2, t + 260000));
		ok &= check("N3 vs N2 right after N2's first REFs", step < 0 ? -step : step, 100);
		ok &= check("spread after it settles", run(2, t + 260000, t + 10e6, all_on, 0), 50);
	}

	printf("3. Relayed REF: N3 only on bus B, N2 relays and re-stamps\n");
	setup();
	ts_set_leader(&nodes[1].ts, true);
	ok &= check("spread including the relayed node", run(1, 0, 10e6, hop, 2), 100);

	printf("\n%s\n", ok ? "all time sync tests passed" : "SOME TIME SYNC TESTS FAILED");
	return ok ? 0 : 1;
}
