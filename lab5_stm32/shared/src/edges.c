/*
 * Blinker lamps and PPS, both timed from global time (part 6).
 *
 * Lamps: a pattern is (mode, start time, half period). The lamps are on for
 * the first half period after start, off for the next, and so on. Both STM32s
 * work the edges out from their own global time, so front and rear line up
 * even when BLINK reached one of them late through a relay (checkoff 9).
 *
 * Where the pattern comes from:
 *   - normally the cockpit's BLINK frame (mode + start time, ~20 ms ahead)
 *   - failure mode (committed COCKPIT_ERROR, or the cockpit just went silent
 *     and we're not NORMAL): our own 4 Hz hazards, starting at global time 0.
 *     Every node picks the same grid, so they're in step without talking,
 *     and stay in step through a leader change (checkoff 6).
 *
 * PPS: rising edge at every whole second of global_gettime(), 100 ms wide.
 * Each edge's numbers go to pps_q and main() prints them (part 6.4).
 *
 * The thread sleeps until ~300 us before the next edge, then busy-waits on
 * the 1 us clock (the kernel tick is 100 us). One thread for both, so two
 * edges at the same instant don't hold each other up.
 */
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "config.h"
#include "edges.h"
#include "gtime.h"
#include "raft_task.h"
#include "tp.h"
#include "zone.h"

K_MSGQ_DEFINE(pps_q, sizeof(struct pps_record), 4, 8);

enum { LAMP_LEFT, LAMP_RIGHT, N_LAMPS };

static const struct gpio_dt_spec lamps[N_LAMPS] = {
	GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), blink_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), blink_gpios, 1),
};

struct pattern {
	uint8_t mode;      /* CAN_BLINK_* */
	uint64_t start_us; /* global time */
	uint32_t half_us;
};

static struct k_spinlock lock;
/* power up in ERROR with hazards, like lab 3 */
static struct pattern cur = {CAN_BLINK_HAZARD, 0, HAZARD_HALF_MS * 1000};
static struct pattern next;
static bool have_next;
static K_SEM_DEFINE(kick, 0, 1);

static const struct pattern fail_pattern = {CAN_BLINK_HAZARD, 0, FAIL_HAZARD_HALF_MS * 1000};

static uint32_t half_for(uint8_t mode)
{
	return (mode == CAN_BLINK_HAZARD ? HAZARD_HALF_MS : BLINK_HALF_MS) * 1000u;
}

static bool same(const struct pattern *a, const struct pattern *b)
{
	return a->mode == b->mode && a->start_us == b->start_us;
}

void blink_on_msg(const struct can_blink *b)
{
	uint64_t g = global_gettime();
	/* BLINK carries 32 bits of ms: put back the high bits, nearest to now */
	uint64_t now_ms = g / 1000;
	uint64_t start_ms = (now_ms & ~0xFFFFFFFFull) | b->start_ms;

	if (start_ms > now_ms + 0x80000000ull && start_ms >= 0x100000000ull) {
		start_ms -= 0x100000000ull;
	} else if (start_ms + 0x80000000ull < now_ms) {
		start_ms += 0x100000000ull;
	}

	struct pattern p = {b->mode, start_ms * 1000, half_for(b->mode)};
	k_spinlock_key_t key = k_spin_lock(&lock);

	if (!same(&p, &cur) && !(have_next && same(&p, &next))) {
		if (p.start_us <= g) {
			cur = p;
			have_next = false;
		} else {
			next = p;
			have_next = true;
		}
	}
	k_spin_unlock(&lock, key);
	k_sem_give(&kick);
}

void blink_kick(void)
{
	k_sem_give(&kick);
}

static bool failure_mode(void)
{
	/* committed COCKPIT_ERROR, or (faster, lab 3) we're not NORMAL and the cockpit went quiet */
	return raft_task_failure_mode() ||
	       (zone_state() != ZONE_NORMAL && !zone_blink_master_alive());
}

/* lamps on/off for pattern p at global time g, and when that changes next */
static void lamp_state(const struct pattern *p, uint64_t g, bool *left, bool *right, uint64_t *next_edge)
{
	*left = *right = false;
	*next_edge = UINT64_MAX;
	if (p->mode == CAN_BLINK_OFF || g < p->start_us) {
		return;
	}
	uint64_t k = (g - p->start_us) / p->half_us;
	bool on = (k % 2) == 0;

	*left = on && (p->mode == CAN_BLINK_LEFT || p->mode == CAN_BLINK_HAZARD);
	*right = on && (p->mode == CAN_BLINK_RIGHT || p->mode == CAN_BLINK_HAZARD);
	*next_edge = p->start_us + (k + 1) * p->half_us;
}

static void edges_thread(void *p1, void *p2, void *p3)
{
	uint64_t pps_rise = 0, pps_fall = 0;
	bool pps_high = false;

	for (;;) {
		uint64_t g = global_gettime();
		struct pattern p;
		bool left, right;
		uint64_t ev;

		/* the pattern in effect now */
		k_spinlock_key_t key = k_spin_lock(&lock);

		if (have_next && g >= next.start_us) {
			cur = next;
			have_next = false;
		}
		p = failure_mode() ? fail_pattern : cur;
		uint64_t pending = have_next ? next.start_us : UINT64_MAX;

		k_spin_unlock(&lock, key);

		lamp_state(&p, g, &left, &right, &ev);
		gpio_pin_set_dt(&lamps[LAMP_LEFT], left);
		gpio_pin_set_dt(&lamps[LAMP_RIGHT], right);
		if (pending < ev) {
			ev = pending;
		}

		/* PPS. Only fire if we're right at the scheduled second: if global
		 * time jumped past it (first sync, new leader) just pick the next one */
		if (!pps_high) {
			if (pps_rise != 0 && g >= pps_rise && g - pps_rise < 2000) {
				uint64_t l = gtime_local_us();
				struct gtime_stats st;
				struct pps_record r;

				tp_set(TP_PPS, true);
				pps_high = true;
				gtime_get_stats(&st);
				r = (struct pps_record){
					.global_us = global_gettime(), .local_us = l,
					.drift_ppb = st.drift_ppb, .last_error_us = st.last_error_us,
					.synced = st.synced, .leader = st.leader,
				};
				(void)k_msgq_put(&pps_q, &r, K_NO_WAIT);
				pps_fall = pps_rise + PPS_HIGH_MS * 1000u;
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

		tp_set(TP_SYNC, gtime_synced());

		/* far away: sleep (in chunks, the time model moves a bit with
		 * every REF), or until a BLINK / state change kicks us */
		uint64_t now = gtime_local_us();
		uint64_t target = gtime_global_to_local(ev);

		if (target > now + 3000) {
			uint64_t wake = target - 2000;

			if (wake > now + 50000) {
				wake = now + 50000;
			}
			(void)k_sem_take(&kick, K_TIMEOUT_ABS_US(wake));
			continue;
		}
		/* close: spin to the exact time, then go round and apply it */
		gtime_wait_until(target);
	}
}

K_THREAD_DEFINE(edges_tid, STACK_SZ, edges_thread, NULL, NULL, NULL, PRIO_EDGES, 0, SYS_FOREVER_MS);

int edges_init(void)
{
	for (int i = 0; i < N_LAMPS; i++) {
		if (!gpio_is_ready_dt(&lamps[i]) || gpio_pin_configure_dt(&lamps[i], GPIO_OUTPUT_INACTIVE) < 0) {
			printk("blinker GPIO not ready\n");
			return -ENODEV;
		}
	}
	k_thread_start(edges_tid);
	return 0;
}
