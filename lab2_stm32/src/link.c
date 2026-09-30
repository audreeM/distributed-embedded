#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include "link.h"
#include "sys_state.h"

#define LINK_UART_NODE DT_ALIAS(link_uart)
#define GAP_MS         2 /* abandon a partial frame after this inter-byte gap */
#define RX_Q_DEPTH     4

static const struct device *const uart = DEVICE_DT_GET(LINK_UART_NODE);
/* CMD_RX test point: toggles on every CRC-valid command frame. */
static const struct gpio_dt_spec tp_cmd_rx =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), cmdrx_gpios);

/*
 * How data moves between contexts (volatile alone is not enough, so each
 * piece of shared state uses a real Zephyr primitive):
 *   UART ISR  --rx_q (k_msgq, copies frames)-->  link_rx thread
 *   link_rx   --latest (spinlock)-->             link_get_cmd() callers
 *   anyone    --wake_sem-->                      the one link_wait() caller
 *   status timer ISR --tx_rb (ring buffer + spinlock)--> UART TX ISR
 *   counters / currents: atomic_t, so single values never tear
 */
K_MSGQ_DEFINE(rx_q, sizeof(struct lp_frame), RX_Q_DEPTH, 4);
/* Binary semaphore (limit 1): several wakes before the consumer runs merge
 * into one, but a wake is never lost.
 */
K_SEM_DEFINE(wake_sem, 0, 1);
RING_BUF_DECLARE(tx_rb, 4 * LP_FRAME_MAX); /* two ticks' worth of STATUS + CURRENTS */

static struct k_spinlock tx_lock;  /* tx_rb: status timer ISR vs UART ISR */
static struct k_spinlock cmd_lock; /* latest: link_rx thread vs readers */
static struct link_cmd latest;
static struct lp_parser parser; /* touched only by the UART ISR, so no lock */

static atomic_t crc_err;   /* malformed frames (CRC / LEN / gap) */
static atomic_t range_err; /* CRC-valid but out-of-range commands */
static atomic_t last_seq;  /* SEQ of the last accepted command, echoed to Pi */
static atomic_t i_motor_l, i_motor_r, i_servo; /* set by Part 3 ADC code */
/* One SEQ counter per message type, as in the Lab 1 catalog (§7.5).
 * Touched only by the status timer.
 */
static uint8_t status_seq, currents_seq;

/* Counters are 8-bit in the status frame; stick at 255 instead of wrapping
 * so a big number never looks like a small one.
 */
static uint8_t sat8(atomic_t *v)
{
	atomic_val_t x = atomic_get(v);

	return x > 255 ? 255 : (uint8_t)x;
}

/* ---------------------------------------------------------------- ISRs */

/*
 * UART interrupt: RX parses bytes into frames, TX feeds status bytes from the
 * ring buffer into the UART. The ISR only does short, bounded work: no
 * blocking, no printk, no allocation. Range checks and state changes happen
 * in the link_rx thread instead.
 */
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev); /* latch the IRQ flags before checking them */

	if (uart_irq_rx_ready(dev)) {
		uint8_t buf[16];
		int n;

		/* Drain everything the UART has so no byte is left behind. */
		while ((n = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
			/* Raw CPU cycle counter: cheap to read in an ISR, and
			 * the parser only needs differences between readings.
			 */
			uint32_t now = k_cycle_get_32();

			for (int i = 0; i < n; i++) {
				enum lp_result r = lp_parse_byte(&parser, buf[i], now);

				if (r == LP_OK && parser.frame.type == LP_TYPE_CMD) {
					/* CMD_RX test point: the moment a complete,
					 * CRC-valid command exists on this MCU. Start
					 * of every R2.x latency measurement.
					 */
					gpio_pin_toggle_dt(&tp_cmd_rx);
					/* Newest wins: drop the oldest queued frame
					 * rather than block or lose the fresh one.
					 */
					if (k_msgq_put(&rx_q, &parser.frame, K_NO_WAIT) != 0) {
						struct lp_frame stale;

						(void)k_msgq_get(&rx_q, &stale, K_NO_WAIT);
						(void)k_msgq_put(&rx_q, &parser.frame, K_NO_WAIT);
					}
				} else if (r == LP_ERR_CRC || r == LP_ERR_LEN || r == LP_ERR_GAP) {
					atomic_inc(&crc_err);
				}
			}
		}
	}

	/* TX: copy as many bytes as the UART FIFO will take. When the ring
	 * buffer is empty, turn the TX interrupt off, or it fires forever.
	 */
	if (uart_irq_tx_ready(dev)) {
		uint8_t *data;
		k_spinlock_key_t key = k_spin_lock(&tx_lock);
		uint32_t len = MIN(ring_buf_get_ptr(&tx_rb, &data, 0), 16U);

		if (len == 0) {
			uart_irq_tx_disable(dev);
		} else {
			int sent = uart_fifo_fill(dev, data, len);

			ring_buf_consume(&tx_rb, sent > 0 ? sent : 0);
		}
		k_spin_unlock(&tx_lock, key);
	}
}

/*
 * Link watchdog. link_rx restarts this one-shot timer on every accepted
 * command, so it only expires after LINK_TIMEOUT_MS with no good command:
 * cable unplugged, Pi program stopped, or proxy dead (the Pi stops sending).
 * Runs in ISR context, so it only sets a fault bit. The actuator code sees
 * the change through link_wait() and does the actual fail-safe.
 */
static void watchdog_expiry(struct k_timer *t)
{
	ARG_UNUSED(t);
	sys_state_set_fault(LP_FAULT_LINK_LOST);
}

/*
 * Every 20 ms: STATUS (heartbeat: state + faults) followed immediately by
 * CURRENTS. Each fits one 8-byte payload, so later each can be its own CAN
 * message. A kernel timer (not a thread with k_sleep) keeps the period
 * locked to the system clock, so it doesn't drift no matter how busy the
 * threads are. The ISR only queues the bytes; the UART TX interrupt sends
 * them (2 x 15 bytes = 2.6 ms of wire time).
 */
static void status_tick(struct k_timer *t)
{
	ARG_UNUSED(t);

	uint32_t faults = sys_state_faults();
	struct lp_status s = {
		.state = faults ? LP_STATE_ERROR : LP_STATE_NORMAL,
		.faults = (uint8_t)faults,
		.last_seq = (uint8_t)atomic_get(&last_seq),
		.crc_err = sat8(&crc_err),
		.range_err = sat8(&range_err),
	};
	struct lp_currents c = {
		.i_motor_l_ma = (uint16_t)atomic_get(&i_motor_l),
		.i_motor_r_ma = (uint16_t)atomic_get(&i_motor_r),
		.i_servo_ma = (uint16_t)atomic_get(&i_servo),
	};
	uint8_t buf[2 * LP_FRAME_MAX];
	size_t n = lp_encode_status(buf, status_seq++, &s);

	n += lp_encode_currents(buf + n, currents_seq++, &c);

	k_spinlock_key_t key = k_spin_lock(&tx_lock);

	/* Queue both frames or neither; if the UART is somehow backed up,
	 * skip this tick rather than send half a frame.
	 */
	if (ring_buf_space_get(&tx_rb) >= n) {
		ring_buf_put(&tx_rb, buf, n);
	}
	k_spin_unlock(&tx_lock, key);
	uart_irq_tx_enable(uart);
}

K_TIMER_DEFINE(link_wd, watchdog_expiry, NULL);
K_TIMER_DEFINE(status_timer, status_tick, NULL);

/* ------------------------------------------------------------- thread */

/*
 * Accept one decoded command: range check, feed the watchdog, publish, run
 * the fault/button logic, wake the control thread. Shared by the UART path
 * (link_rx thread) and the test shell (link_inject_cmd). The mutex keeps the
 * two from running sys_state_on_buttons() at the same time.
 */
K_MUTEX_DEFINE(accept_mtx);

static void accept_cmd(uint8_t seq, const struct lp_cmd *cmd)
{
	k_mutex_lock(&accept_mtx, K_FOREVER);

	if (!lp_cmd_in_range(cmd)) {
		/* Reject: never stored, never acted on, and it does not
		 * feed the link watchdog.
		 */
		atomic_inc(&range_err);
		sys_state_on_bad_cmd();
		k_mutex_unlock(&accept_mtx);
		return;
	}

	/* Good command -> push the watchdog deadline out again. */
	k_timer_start(&link_wd, K_MSEC(LINK_TIMEOUT_MS), K_NO_WAIT);

	/* Publish as one struct under a lock, so a reader never sees
	 * a new throttle paired with an old brake.
	 */
	int64_t now = k_uptime_get();
	k_spinlock_key_t key = k_spin_lock(&cmd_lock);

	latest = (struct link_cmd){
		.valid = true,
		.seq = seq,
		.throttle = cmd->throttle,
		.brake = cmd->brake,
		.steer = cmd->steer,
		.buttons = cmd->buttons,
		.rx_ms = now,
	};
	k_spin_unlock(&cmd_lock, key);
	atomic_set(&last_seq, seq);

	sys_state_on_buttons(cmd->buttons, now); /* self-test press logic */
	sys_state_on_good_cmd(); /* may clear POWERUP/LINK_LOST/BAD_CMD */
	k_mutex_unlock(&accept_mtx);
	link_notify();           /* wake the control thread */
}

/*
 * link_rx thread (priority LINK_RX_PRIO = 1, above everything else in the
 * app): validates each frame from the ISR and publishes it. It is high
 * priority because brake/throttle (2 ms budget) go through here.
 */
static void link_rx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct lp_frame f;
	struct lp_cmd cmd;

	for (;;) {
		k_msgq_get(&rx_q, &f, K_FOREVER);
		lp_decode_cmd(&f, &cmd);
		accept_cmd(f.seq, &cmd);
	}
}

/* Priority is set here. It starts at boot, but sits blocked on rx_q until
 * link_init() enables the UART interrupt.
 */
K_THREAD_DEFINE(link_rx_tid, 1024, link_rx_thread, NULL, NULL, NULL, LINK_RX_PRIO, 0, 0);

/* ---------------------------------------------------------------- API */

int link_init(void)
{
	int ret;

	if (!device_is_ready(uart) || !gpio_is_ready_dt(&tp_cmd_rx)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&tp_cmd_rx, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	/* Inter-byte gap limit converted to CPU cycles (84 MHz -> 168000). */
	lp_parser_init(&parser, (uint32_t)(sys_clock_hw_cycles_per_sec() / 1000U * GAP_MS));

	ret = uart_irq_callback_user_data_set(uart, uart_isr, NULL);
	if (ret < 0) {
		return ret;
	}
	/* Flush anything that arrived before we were listening. */
	uint8_t junk;

	while (uart_fifo_read(uart, &junk, 1) > 0) {
	}
	uart_irq_rx_enable(uart);

	/* The watchdog is NOT started here: at boot we are already in ERROR
	 * (POWERUP), and it starts with the first good command.
	 */
	k_timer_start(&status_timer, K_MSEC(LINK_STATUS_MS), K_MSEC(LINK_STATUS_MS));
	return 0;
}

void link_get_cmd(struct link_cmd *out)
{
	k_spinlock_key_t key = k_spin_lock(&cmd_lock);

	*out = latest;
	k_spin_unlock(&cmd_lock, key);
}

bool link_wait(k_timeout_t timeout)
{
	return k_sem_take(&wake_sem, timeout) == 0;
}

void link_set_currents(uint16_t motor_l_ma, uint16_t motor_r_ma, uint16_t servo_ma)
{
	atomic_set(&i_motor_l, motor_l_ma);
	atomic_set(&i_motor_r, motor_r_ma);
	atomic_set(&i_servo, servo_ma);
}

void link_inject_cmd(uint8_t seq, const struct lp_cmd *cmd)
{
	accept_cmd(seq, cmd);
}

void link_notify(void)
{
	k_sem_give(&wake_sem);
}
