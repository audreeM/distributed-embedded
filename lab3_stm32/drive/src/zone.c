#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "blink.h"
#include "canbus.h"
#include "config.h"
#include "motor.h"
#include "tp.h"
#include "zone.h"

#define ZONES (CAN_ZONE_N3 + 1) 

static struct k_spinlock lock;
static atomic_t state = ATOMIC_INIT(ZONE_ERROR);

static struct {
	uint32_t start_ms;
	uint32_t last_rx_ms[BUS_COUNT];          
	uint32_t last_seen_ms[ZONES][BUS_COUNT];  
	uint32_t last_hb_ms[ZONES][BUS_COUNT];   
	uint32_t last_ctrl_ok_ms;
	int good_ctrl;                           
	uint32_t fault_ms[ZONES];                
	bool self_test;
	/* HB_MON outputs */
	bool degraded[BUS_COUNT];
	bool isolated;
	bool alive[ZONES];
} z;

static atomic_t fault_to_send;
static uint8_t fault_seq;
static uint32_t last_fault_tx_ms;
static K_MUTEX_DEFINE(fault_tx_lock);

static atomic_t wdt_bits;

/* 0 means "never": measure from boot then. */
static uint32_t silent_ms(uint32_t last, uint32_t now)
{
	return now - (last ? last : z.start_ms);
}

static bool set_state(enum zone_state s)
{
	if (atomic_set(&state, s) != s) {
		tp_set(TP_ERR, s != ZONE_NORMAL);
		return true;
	}
	return false;
}

static void kick_outputs(void)
{
	motor_kick();
	blink_kick();
}

static bool to_error_locked(void)
{
	z.good_ctrl = 0;
	return atomic_get(&state) != ZONE_LOCAL_FAIL && set_state(ZONE_ERROR);
}

enum zone_state zone_state(void)
{
	return (enum zone_state)atomic_get(&state);
}

uint8_t zone_wire_state(void)
{
	return zone_state() == ZONE_NORMAL ? CAN_STATE_NORMAL : CAN_STATE_ERROR;
}

uint8_t zone_health_bits(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint8_t bits = (z.degraded[BUS_A] ? CAN_HEALTH_A_DEGRADED : 0) |
		       (z.degraded[BUS_B] ? CAN_HEALTH_B_DEGRADED : 0);

	k_spin_unlock(&lock, key);
	return bits;
}

static void send_pending_fault(void)
{
	uint32_t now = k_uptime_get_32();

	k_mutex_lock(&fault_tx_lock, K_FOREVER);
	if (atomic_get(&fault_to_send) != 0 && now - last_fault_tx_ms >= FAULT_MIN_GAP_MS) {
		struct can_fault f = {
			.zone = CAN_ZONE_N3, .reason = (uint8_t)atomic_clear(&fault_to_send),
		};
		uint8_t d[CAN_DLC];

		can_pack_fault(d, fault_seq, &f);
		canbus_send_both(CAN_ID_FAULT, d, &fault_seq);
		last_fault_tx_ms = now;
	}
	k_mutex_unlock(&fault_tx_lock);
}

void zone_on_frame(int bus, uint8_t sender, bool is_hb)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&lock);

	z.last_rx_ms[bus] = now;
	if (sender != 0 && sender < ZONES) {
		z.last_seen_ms[sender][bus] = now;
		if (is_hb) {
			z.last_hb_ms[sender][bus] = now;
		}
	}
	k_spin_unlock(&lock, key);
}

void zone_on_ctrl(enum can_check result)
{
	bool changed = false;
	k_spinlock_key_t key = k_spin_lock(&lock);

	if (result == CAN_OK) {
		z.last_ctrl_ok_ms = k_uptime_get_32();
		z.good_ctrl++;
	} else {
		/* catalog: a broken or out-of-range CTRL -> ERROR and send FAULT */
		atomic_set(&fault_to_send, result == CAN_OUT_OF_RANGE ? CAN_FAULT_BAD_VALUE
								       : CAN_FAULT_MALFORMED);
		changed = to_error_locked();
	}
	k_spin_unlock(&lock, key);
	if (changed) {
		kick_outputs();
	}
}

void zone_on_fault(const struct can_fault *f)
{
	if (f->zone == CAN_ZONE_N3) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&lock);

	z.fault_ms[f->zone] = k_uptime_get_32() | 1; /* never 0 */
	bool changed = to_error_locked();

	k_spin_unlock(&lock, key);
	if (changed) {
		kick_outputs();
	}
}

bool zone_relay_to(uint8_t sender, int from_bus, int *to_bus)
{
	int other = from_bus == BUS_A ? BUS_B : BUS_A;
	uint32_t now = k_uptime_get_32();
	bool relay;

	if (sender == 0 || sender >= ZONES || zone_state() == ZONE_LOCAL_FAIL) {
		return false;
	}
	k_spinlock_key_t key = k_spin_lock(&lock);

	relay = silent_ms(z.last_seen_ms[sender][other], now) >= RELAY_AFTER_MS &&
		silent_ms(z.last_rx_ms[other], now) < DEGRADED_MS;
	k_spin_unlock(&lock, key);
	*to_bus = other;
	return relay;
}

bool zone_blink_master_alive(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool alive = z.alive[CAN_ZONE_N1] && !z.isolated;

	k_spin_unlock(&lock, key);
	return alive;
}

void zone_wdt_checkin(uint32_t bit)
{
	atomic_or(&wdt_bits, bit);
}

uint32_t zone_wdt_take(void)
{
	return (uint32_t)atomic_clear(&wdt_bits);
}

static void hb_mon_update(void)
{
	const uint32_t hb_limit = HB_MISS_LIMIT * HB_PERIOD_MS;
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool failed = false, alive = true, fault_hold = false, changed = false;

	/* R4.5: a zone has failed when HB_MISS_LIMIT heartbeats are missed on BOTH buses. */
	for (int k = CAN_ZONE_N1; k <= CAN_ZONE_N2; k += 1) { /* N1 and N3 */
		bool fresh_a = silent_ms(z.last_hb_ms[k][BUS_A], now) < hb_limit;
		bool fresh_b = silent_ms(z.last_hb_ms[k][BUS_B], now) < hb_limit;

		z.alive[k] = fresh_a || fresh_b;
		alive &= z.alive[k];
		failed |= !z.alive[k];

		if (z.fault_ms[k] != 0) {
			if (now - z.fault_ms[k] >= FAULT_HOLD_MS && z.alive[k]) {
				z.fault_ms[k] = 0;
			} else {
				fault_hold = true;
			}
		}
	}

	bool quiet_a = silent_ms(z.last_rx_ms[BUS_A], now) >= DEGRADED_MS;
	bool quiet_b = silent_ms(z.last_rx_ms[BUS_B], now) >= DEGRADED_MS;

	z.isolated = quiet_a && quiet_b;
	z.degraded[BUS_A] = quiet_a && !quiet_b;
	z.degraded[BUS_B] = quiet_b && !quiet_a;

	bool ctrl_fresh = z.last_ctrl_ok_ms != 0 && now - z.last_ctrl_ok_ms < CTRL_STALE_MS;

	switch (zone_state()) {
	case ZONE_LOCAL_FAIL:
		if (!z.self_test) {
			z.good_ctrl = 0;
			changed = set_state(ZONE_ERROR);
		}
		break;
	case ZONE_NORMAL:
		if (failed || z.isolated || !ctrl_fresh || fault_hold) {
			if (failed && atomic_get(&fault_to_send) == 0) {
				atomic_set(&fault_to_send, CAN_FAULT_HB_LOST);
			}
			changed = to_error_locked();
		}
		break;
	case ZONE_ERROR:
		if (alive && !z.isolated && !fault_hold && ctrl_fresh &&
		    z.good_ctrl >= GOOD_CTRL_TO_CLEAR) {
			changed = set_state(ZONE_NORMAL);
		}
		break;
	}

	tp_set(TP_DEG_A, z.degraded[BUS_A]);
	tp_set(TP_DEG_B, z.degraded[BUS_B]);
	k_spin_unlock(&lock, key);
	if (changed) {
		kick_outputs();
	}
}

K_TIMER_DEFINE(hb_mon_timer, NULL, NULL);

static void hb_mon_thread(void *p1, void *p2, void *p3)
{
	k_timer_start(&hb_mon_timer, K_MSEC(HB_PERIOD_MS), K_MSEC(HB_PERIOD_MS));
	for (;;) {
		k_timer_status_sync(&hb_mon_timer);
		hb_mon_update();
		send_pending_fault();
		zone_wdt_checkin(WDT_HB_MON);
	}
}

K_THREAD_DEFINE(hb_mon_tid, STACK_SZ, hb_mon_thread, NULL, NULL, NULL, PRIO_HB_MON, 0,
		SYS_FOREVER_MS);


		/* TEST_BTN*/

static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback btn_cb;
static K_SEM_DEFINE(btn_sem, 0, 1);

/* GPIO ISR: only wake the thread (no SPI, logging or allocation in an ISR). */
static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&btn_sem);
}

static void test_btn_thread(void *p1, void *p2, void *p3)
{
	int64_t lockout_until = 0;
	int64_t first_press = -1;

	for (;;) {
		k_sem_take(&btn_sem, K_FOREVER);
		int64_t now = k_uptime_get();

		if (now < lockout_until) {
			continue;
		}
		lockout_until = now + SELFTEST_LOCKOUT_MS;

		k_spinlock_key_t key = k_spin_lock(&lock);

		if (!z.self_test) {
			z.self_test = true;
			first_press = -1;
			atomic_set(&fault_to_send, CAN_FAULT_SELF_TEST);
			set_state(ZONE_LOCAL_FAIL); 
			k_spin_unlock(&lock, key);
			kick_outputs();
			send_pending_fault();       
			continue;
		}
		if (first_press >= 0 && now - first_press <= SELFTEST_DOUBLE_MS) {
			z.self_test = false;        
			first_press = -1;
		} else {
			first_press = now;
		}
		k_spin_unlock(&lock, key);
	}
}

K_THREAD_DEFINE(test_btn_tid, STACK_SZ, test_btn_thread, NULL, NULL, NULL, PRIO_TEST_BTN, 0,
		SYS_FOREVER_MS);

int zone_init(void)
{
	z.start_ms = k_uptime_get_32();

	gpio_init_callback(&btn_cb, btn_isr, BIT(btn.pin));
	if (!gpio_is_ready_dt(&btn) || gpio_pin_configure_dt(&btn, GPIO_INPUT) < 0 ||
	    gpio_add_callback(btn.port, &btn_cb) < 0 ||
	    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE) < 0) {
		printk("test button not ready\n");
	}
	k_thread_start(hb_mon_tid);
	k_thread_start(test_btn_tid);
	return 0;
}