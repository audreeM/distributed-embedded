/*
 * PPS + HZ thread, see edges.h. Same idea as the STM32's edges.c: sleep until
 * just before the next edge (clock_nanosleep), then busy-wait the last
 * EDGE_SPIN_US on CLOCK_MONOTONIC, then set the pin. SCHED_FIFO above the
 * main loop so the edge isn't held up by CAN or UDP work.
 */
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <time.h>

#include "can_catalog.h"
#include "config.h"
#include "edges.h"
#include "gpio_tp.h"
#include "gtime.h"
#include "util.h"

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t pat_mode = CAN_BLINK_HAZARD;
static uint64_t pat_start, pat_half = 250000;
static struct pps_record pps_last;
static bool pps_new;

static pthread_t tid;
static volatile bool running;

void edges_set_pattern(uint8_t mode, uint64_t start_us, uint64_t half_us)
{
	pthread_mutex_lock(&lock);
	pat_mode = mode;
	pat_start = start_us;
	pat_half = half_us;
	pthread_mutex_unlock(&lock);
}

bool edges_take_pps(struct pps_record *r)
{
	bool got;

	pthread_mutex_lock(&lock);
	got = pps_new;
	*r = pps_last;
	pps_new = false;
	pthread_mutex_unlock(&lock);
	return got;
}

static void sleep_until_local(uint64_t t_us)
{
	struct timespec ts = {.tv_sec = (time_t)(t_us / 1000000u), .tv_nsec = (long)(t_us % 1000000u) * 1000};

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

static void *edges_thread(void *arg)
{
	uint64_t pps_rise = 0, pps_fall = 0;
	bool pps_high = false;

	(void)arg;
	while (running) {
		uint64_t g = global_gettime();
		uint8_t mode;
		uint64_t start, half, ev = UINT64_MAX;

		pthread_mutex_lock(&lock);
		mode = pat_mode;
		start = pat_start;
		half = pat_half;
		pthread_mutex_unlock(&lock);

		/* HZ: on during the "on" half periods of a hazard pattern */
		bool hz = false;

		if (mode == CAN_BLINK_HAZARD && g >= start && half > 0) {
			uint64_t k = (g - start) / half;

			hz = (k % 2) == 0;
			ev = start + (k + 1) * half;
		} else if (mode == CAN_BLINK_HAZARD) {
			ev = start;
		}
		tp_set(TP_HZ, hz);

		/* PPS (only fire right at the scheduled second, see the STM32 version) */
		if (!pps_high) {
			if (pps_rise != 0 && g >= pps_rise && g - pps_rise < 2000) {
				uint64_t l = now_us();
				struct gtime_stats st;

				tp_set(TP_PPS, true);
				pps_high = true;
				gtime_get_stats(&st);
				pthread_mutex_lock(&lock);
				pps_last = (struct pps_record){
					.global_us = global_gettime(), .local_us = l,
					.drift_ppb = st.drift_ppb, .last_error_us = st.last_error_us,
					.synced = st.synced, .leader = st.leader,
				};
				pps_new = true;
				pthread_mutex_unlock(&lock);
				pps_fall = pps_rise + MS(PPS_HIGH_MS);
			} else if (pps_rise == 0 || g >= pps_rise || pps_rise - g > 1000000u) {
				pps_rise = (g / 1000000u + 1) * 1000000u;
			}
		} else if (g >= pps_fall) {
			tp_set(TP_PPS, false);
			pps_high = false;
			pps_rise = (g / 1000000u + 1) * 1000000u;
		}
		uint64_t pps_ev = pps_high ? pps_fall : pps_rise;

		if (pps_ev < ev) {
			ev = pps_ev;
		}

		/* far away: sleep in chunks (the time model shifts a little with
		 * every REF, and the pattern can change). Close: spin. */
		uint64_t now = now_us();
		uint64_t target = gtime_global_to_local(ev);

		if (target > now + 3000) {
			uint64_t wake = target - 2000;

			if (wake > now + 20000) {
				wake = now + 20000;
			}
			sleep_until_local(wake);
			continue;
		}
		if (target > now + EDGE_SPIN_US) {
			sleep_until_local(target - EDGE_SPIN_US);
		}
		while (now_us() < target) {
		}
	}
	return NULL;
}

int edges_start(void)
{
	pthread_attr_t attr;
	struct sched_param sp = {.sched_priority = EDGE_RT_PRIORITY};

	running = true;
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setschedparam(&attr, &sp);
	if (pthread_create(&tid, &attr, edges_thread, NULL) != 0) {
		/* no realtime (not root): run it as a normal thread */
		perror("edges thread SCHED_FIFO (run with sudo)");
		if (pthread_create(&tid, NULL, edges_thread, NULL) != 0) {
			perror("edges thread");
			running = false;
			return -1;
		}
	}
	pthread_attr_destroy(&attr);
	return 0;
}

void edges_stop(void)
{
	if (running) {
		running = false;
		pthread_join(tid, NULL);
	}
}
