/*
 * Lab 2 STM32 zone.
 *
 * Part 2 (link.c, sys_state.c) is complete. The loop below is a placeholder
 * that prints the link state for the Part 2 checkpoint and shows the fault
 * state on LD2; Part 3 actuator code replaces or extends it.
 *
 * Contract for Part 3 code:
 *   - sys_state_is_error() true  -> motor PWM off + dynamic brake, hazards 2 Hz
 *   - otherwise use link_get_cmd() (throttle/brake 0..100, steer -100..100)
 *   - brake > 0 wins over throttle
 *   - report currents with link_set_currents()
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "config.h"
#include "state.h"
#include "link.h"
#include "sys_state.h"

#define PRINT_PERIOD_MS 200
#define LED_ERROR_HALF_MS 125 /* 4 Hz blink on LD2 while in ERROR */

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void print_faults(uint32_t f)
{
	printk("[%6u ms] %s faults=0x%02x%s%s%s%s\n", (uint32_t)k_uptime_get(),
	       f ? "ERROR " : "NORMAL", f,
	       (f & LP_FAULT_POWERUP) ? " POWERUP" : "",
	       (f & LP_FAULT_LINK_LOST) ? " LINK_LOST" : "",
	       (f & LP_FAULT_BAD_CMD) ? " BAD_CMD" : "",
	       (f & LP_FAULT_SELF_TEST) ? " SELF_TEST" : "");
}

int main(void)
{
	state_init();
	int ret;

	if (!gpio_is_ready_dt(&led) || gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) < 0) {
		return 0;
	}

	// sests up frame receiving
	ret = link_init();
	if (ret < 0) {
		printk("link_init failed: %d\n", ret);
		return 0;
	}

	// wakes up when a frame arrives, serves brake, throttle, servo, and button presses
	k_thread_start(cmd_tid);

	// every 10 ms reads the encoders and updates the motors
	k_thread_start(motor_tid);

	// every 10 ms reads the three current sensors
	k_thread_start(sense_tid);

	printk("lab2_stm32: link on USART6 (PA11 TX / PA12 RX), timeout %d ms\n",
	       LINK_TIMEOUT_MS);

	uint32_t prev_faults = UINT32_MAX;
	int64_t next_print = 0;
	int64_t next_led = 0;
	struct link_cmd c;

	for (;;) {
		/* Wakes early on a new command or a fault change; the 10 ms
		 * timeout keeps the LED blinking even with no traffic.
		 */
		link_wait(K_MSEC(10));

		int64_t now = k_uptime_get();
		uint32_t f = sys_state_faults();

		if (f != prev_faults) {
			print_faults(f);
			prev_faults = f;
		}

		if (now >= next_print) {
			next_print = now + PRINT_PERIOD_MS;
			link_get_cmd(&c);
			if (c.valid) {
				printk("cmd seq=%3u thr=%3u brk=%3u steer=%+4d btn=0x%02x age=%u ms\n",
				       c.seq, c.throttle, c.brake, c.steer, c.buttons,
				       (uint32_t)(now - c.rx_ms));
			}
		}

		/* LD2: solid = NORMAL, fast blink = ERROR. */
		if (!f) {
			gpio_pin_set_dt(&led, 1);
		} else if (now >= next_led) {
			next_led = now + LED_ERROR_HALF_MS;
			gpio_pin_toggle_dt(&led);
		}
	}
	return 0;
}
