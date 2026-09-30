/**
 * @file cli.c
 * @brief Serial test shell on the ST-Link COM port (USART2, 115200).
 *
 *   status                      faults, latest command, servo pulse
 *   log on|off                  periodic prints (command + motor) on/off
 *
 *   servo us <500..2500>        calibration: raw pulse width, ignores wheel
 *   servo auto                  back to following the wheel / sim steer
 *
 *   sim on|off                  act as the Pi: send a command every 20 ms
 *   sim thr <0..100>            throttle %
 *   sim brake <0..100>          brake %
 *   sim steer <-100..100>       wheel angle
 *   sim btn <0..255>            button bits (LP_BTN_*)
 *   sim bad                     send ONE out-of-range command (checkoff 8)
 *
 * `sim` commands go through link_inject_cmd(): the same range check,
 * watchdog and fault logic as a real UART frame. Use it with the Pi link
 * idle (proxy stopped or cable out). `sim off` stops the stream, so the
 * link watchdog fires -> LINK_LOST, exactly like pulling the cable.
 */
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include "config.h"
#include "link.h"
#include "sys_state.h"
#include "servo.h"
#include "cli.h"

#define SIM_PERIOD_MS 20

static atomic_t log_on;
static atomic_t sim_on;
static atomic_t sim_thr, sim_brk, sim_steer, sim_btn;

bool cli_log_enabled(void)
{
	return atomic_get(&log_on) != 0;
}

/* Parse an integer in [lo, hi]; prints an error and returns false if not. */
static bool parse_int(const struct shell *sh, const char *s, long lo, long hi, long *out)
{
	char *end;
	long v = strtol(s, &end, 0);

	if (*end != '\0' || v < lo || v > hi) {
		shell_error(sh, "expected an integer %ld..%ld", lo, hi);
		return false;
	}
	*out = v;
	return true;
}

/* ------------------------------------------------------------ general */

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct link_cmd c;
	uint32_t f = sys_state_faults();

	link_get_cmd(&c);
	shell_print(sh, "%s faults=0x%02x | cmd %s seq=%u thr=%u brk=%u steer=%d btn=0x%02x"
		    " | servo=%d us | sim=%s",
		    f ? "ERROR " : "NORMAL", f, c.valid ? "valid" : "none",
		    c.seq, c.throttle, c.brake, c.steer, c.buttons,
		    servo_get_us(), atomic_get(&sim_on) ? "on" : "off");
	return 0;
}

static int cmd_log(const struct shell *sh, size_t argc, char **argv)
{
	atomic_set(&log_on, strcmp(argv[1], "on") == 0);
	return 0;
}

/* -------------------------------------------------------------- servo */

static int cmd_servo_us(const struct shell *sh, size_t argc, char **argv)
{
	long us;

	if (!parse_int(sh, argv[1], SERVO_ABS_MIN_US, SERVO_ABS_MAX_US, &us)) {
		return -EINVAL;
	}
	servo_set_raw_us((int)us);
	shell_warn(sh, "servo raw %ld us - buzzing adithi? What is that bruh", us);
	return 0;
}

static int cmd_servo_auto(const struct shell *sh, size_t argc, char **argv)
{
	servo_set_raw_us(0);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,
	SHELL_CMD_ARG(us, NULL, "Raw pulse width <500..2500>", cmd_servo_us, 2, 0),
	SHELL_CMD_ARG(auto, NULL, "Follow the wheel again", cmd_servo_auto, 1, 0),
	SHELL_SUBCMD_SET_END
);

/* ---------------------------------------------------------------- sim */

static void sim_send(uint8_t thr, uint8_t brk, int8_t steer, uint8_t btn)
{
	static uint8_t seq;
	struct lp_cmd c = {
		.throttle = thr,
		.brake = brk,
		.steer = steer,
		.buttons = btn,
	};

	link_inject_cmd(seq++, &c);
}

static int cmd_sim_on(const struct shell *sh, size_t argc, char **argv)
{
	atomic_set(&sim_on, 1);
	shell_warn(sh, "sim ON: shell is acting as the Pi. Keep the real link idle.");
	return 0;
}

static int cmd_sim_off(const struct shell *sh, size_t argc, char **argv)
{
	atomic_set(&sim_on, 0);
	return 0;
}

static int cmd_sim_thr(const struct shell *sh, size_t argc, char **argv)
{
	long v;

	if (!parse_int(sh, argv[1], 0, 100, &v)) {
		return -EINVAL;
	}
	atomic_set(&sim_thr, v);
	return 0;
}

static int cmd_sim_brake(const struct shell *sh, size_t argc, char **argv)
{
	long v;

	if (!parse_int(sh, argv[1], 0, 100, &v)) {
		return -EINVAL;
	}
	atomic_set(&sim_brk, v);
	return 0;
}

static int cmd_sim_steer(const struct shell *sh, size_t argc, char **argv)
{
	long v;

	if (!parse_int(sh, argv[1], -100, 100, &v)) {
		return -EINVAL;
	}
	atomic_set(&sim_steer, v);
	return 0;
}

static int cmd_sim_btn(const struct shell *sh, size_t argc, char **argv)
{
	long v;

	if (!parse_int(sh, argv[1], 0, 255, &v)) {
		return -EINVAL;
	}
	atomic_set(&sim_btn, v);
	return 0;
}

static int cmd_sim_bad(const struct shell *sh, size_t argc, char **argv)
{
	/* Throttle 200 % is out of range: the link must refuse it. */
	sim_send(200, 0, 0, 0);
	shell_print(sh, "sent throttle=200 (out of range) - expect BAD_CMD");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sim,
	SHELL_CMD_ARG(on, NULL, "Start sending commands every 20 ms", cmd_sim_on, 1, 0),
	SHELL_CMD_ARG(off, NULL, "Stop (link watchdog will fire)", cmd_sim_off, 1, 0),
	SHELL_CMD_ARG(thr, NULL, "Throttle 0..100", cmd_sim_thr, 2, 0),
	SHELL_CMD_ARG(brake, NULL, "Brake 0..100", cmd_sim_brake, 2, 0),
	SHELL_CMD_ARG(steer, NULL, "Steer -100..100", cmd_sim_steer, 2, 0),
	SHELL_CMD_ARG(btn, NULL, "Button bits 0..255", cmd_sim_btn, 2, 0),
	SHELL_CMD_ARG(bad, NULL, "Send one out-of-range command", cmd_sim_bad, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(status, NULL, "Faults, last command, servo", cmd_status, 1, 0);
SHELL_CMD_ARG_REGISTER(log, NULL, "Periodic prints: on|off", cmd_log, 2, 0);
SHELL_CMD_REGISTER(servo, &sub_servo, "Steering servo test", NULL);
SHELL_CMD_REGISTER(sim, &sub_sim, "Act as the Pi (bench testing)", NULL);

/* Stand-in for the Pi: one command every 20 ms while `sim on`. */
static void sim_thread(void *p1, void *p2, void *p3)
{
	for (;;) {
		k_msleep(SIM_PERIOD_MS);
		if (atomic_get(&sim_on)) {
			sim_send((uint8_t)atomic_get(&sim_thr), (uint8_t)atomic_get(&sim_brk),
				 (int8_t)atomic_get(&sim_steer), (uint8_t)atomic_get(&sim_btn));
		}
	}
}

K_THREAD_DEFINE(sim_tid, 1024, sim_thread, NULL, NULL, NULL, PRIO_SIM, 0, 0);