/*
 * Host-side unit test for link_proto.h. Runs on the Mac or the Pi:
 *   gcc -Wall -Wextra -o test_link_proto test_link_proto.c && ./test_link_proto
 */
#include <stdio.h>
#include <string.h>

#include "link_proto.h"

#define GAP 2000 /* us */

static int failures;

#define CHECK(cond, msg)                                         \
	do {                                                     \
		if (!(cond)) {                                   \
			printf("FAIL %s:%d %s\n", __FILE__, __LINE__, msg); \
			failures++;                              \
		}                                                \
	} while (0)

struct tally {
	int ok, crc, len, gap;
	struct lp_frame last;
};

/* Feed bytes spaced dt us apart, starting at *t. */
static void feed(struct lp_parser *p, struct tally *t, const uint8_t *b, size_t n,
		 uint32_t *now, uint32_t dt)
{
	for (size_t i = 0; i < n; i++) {
		*now += dt;
		switch (lp_parse_byte(p, b[i], *now)) {
		case LP_OK:
			t->ok++;
			t->last = p->frame;
			break;
		case LP_ERR_CRC:
			t->crc++;
			break;
		case LP_ERR_LEN:
			t->len++;
			break;
		case LP_ERR_GAP:
			t->gap++;
			break;
		default:
			break;
		}
	}
}

int main(void)
{
	struct lp_parser p;
	struct tally t;
	uint8_t f[LP_FRAME_MAX], g[LP_FRAME_MAX];
	uint32_t now = 0;
	struct lp_cmd c = {.throttle = 42, .brake = 0, .steer = -77, .buttons = LP_BTN_LEFT};
	struct lp_cmd out;

	/* CRC-16/CCITT-FALSE check value */
	CHECK(lp_crc16((const uint8_t *)"123456789", 9) == 0x29B1, "crc check value");

	size_t n = lp_encode_cmd(f, 7, &c);
	CHECK(n == LP_HDR_LEN + LP_PAYLOAD_LEN + LP_CRC_LEN && n == LP_FRAME_MAX,
	      "cmd frame size");

	/* 1. good frame round-trips */
	lp_parser_init(&p, GAP);
	memset(&t, 0, sizeof(t));
	feed(&p, &t, f, n, &now, 87);
	lp_decode_cmd(&t.last, &out);
	CHECK(t.ok == 1 && t.last.seq == 7 && out.throttle == 42 && out.steer == -77 &&
	      out.buttons == LP_BTN_LEFT, "good frame");

	/* 2. garbage (including stray sync bytes) before a frame */
	uint8_t junk[] = {0x00, 0xA5, 0x13, 0xA5, 0xA5, 0xFF, 0x5A};
	memset(&t, 0, sizeof(t));
	feed(&p, &t, junk, sizeof(junk), &now, 87);
	now += 10000;
	feed(&p, &t, f, n, &now, 87);
	CHECK(t.ok == 1, "garbage then frame");

	/* 3. byte dropped mid-frame, next frame 5 ms later is still accepted */
	uint8_t dropped[LP_FRAME_MAX];
	memcpy(dropped, f, 6);
	memcpy(dropped + 6, f + 7, n - 7);
	memset(&t, 0, sizeof(t));
	feed(&p, &t, dropped, n - 1, &now, 87);
	now += 5000;
	feed(&p, &t, f, n, &now, 87);
	CHECK(t.ok == 1 && t.gap == 1, "dropped byte then good frame");

	/* 4. truncated frame (unplugged mid-frame), then a good frame */
	memset(&t, 0, sizeof(t));
	feed(&p, &t, f, 6, &now, 87);
	now += 20000;
	feed(&p, &t, f, n, &now, 87);
	CHECK(t.ok == 1 && t.gap == 1, "truncated then good frame");

	/* 5. corrupted payload -> CRC error, not accepted */
	memcpy(g, f, n);
	g[6] ^= 0x01;
	memset(&t, 0, sizeof(t));
	now += 10000;
	feed(&p, &t, g, n, &now, 87);
	CHECK(t.ok == 0 && t.crc == 1, "bad crc rejected");

	/* 6. LEN that does not match TYPE */
	memcpy(g, f, n);
	g[3] = 9;
	memset(&t, 0, sizeof(t));
	now += 10000;
	feed(&p, &t, g, n, &now, 87);
	CHECK(t.ok == 0 && t.len == 1, "bad len rejected");

	/* 7. back-to-back frames with no gap */
	memset(&t, 0, sizeof(t));
	now += 10000;
	feed(&p, &t, f, n, &now, 87);
	feed(&p, &t, f, n, &now, 87);
	CHECK(t.ok == 2, "back to back");

	/* 8. STATUS then CURRENTS back to back, as the STM32 sends them */
	struct lp_status s = {.state = LP_STATE_NORMAL, .faults = 0, .last_seq = 200,
			      .crc_err = 3, .range_err = 1}, so;
	struct lp_currents cu = {.i_motor_l_ma = 1234, .i_motor_r_ma = 65535,
				 .i_servo_ma = 7}, co;
	uint8_t both[2 * LP_FRAME_MAX];
	n = lp_encode_status(both, 9, &s);
	n += lp_encode_currents(both + n, 4, &cu);
	CHECK(n == 2 * LP_FRAME_MAX, "status + currents size");
	memset(&t, 0, sizeof(t));
	now += 10000;
	feed(&p, &t, both, LP_FRAME_MAX, &now, 87);
	lp_decode_status(&t.last, &so);
	CHECK(t.ok == 1 && t.last.type == LP_TYPE_STATUS && so.last_seq == 200 &&
	      so.crc_err == 3 && so.range_err == 1, "status round trip");
	feed(&p, &t, both + LP_FRAME_MAX, LP_FRAME_MAX, &now, 87);
	lp_decode_currents(&t.last, &co);
	CHECK(t.ok == 2 && t.last.type == LP_TYPE_CURRENTS && t.last.seq == 4 &&
	      co.i_motor_l_ma == 1234 && co.i_motor_r_ma == 65535 && co.i_servo_ma == 7,
	      "currents round trip");

	/* 9. range checks */
	struct lp_cmd r = c;
	CHECK(lp_cmd_in_range(&r), "in range");
	r.throttle = 101;
	CHECK(!lp_cmd_in_range(&r), "throttle > 100");
	r = c;
	r.steer = -101;
	CHECK(!lp_cmd_in_range(&r), "steer < -100");
	r = c;
	r.buttons = LP_BTN_LEFT | LP_BTN_RIGHT;
	CHECK(!lp_cmd_in_range(&r), "left and right");
	r = c;
	r.buttons = 0x80;
	CHECK(!lp_cmd_in_range(&r), "reserved button bit");
	r = c;
	r.reserved[2] = 1;
	CHECK(!lp_cmd_in_range(&r), "reserved payload byte");

	/* 10. timestamp wrap-around does not look like a gap */
	lp_parser_init(&p, GAP);
	memset(&t, 0, sizeof(t));
	now = 0xFFFFFF00u;
	n = lp_encode_cmd(f, 1, &c);
	feed(&p, &t, f, n, &now, 87);
	CHECK(t.ok == 1 && t.gap == 0, "timestamp wrap");

	printf("%s (%d failure%s)\n", failures ? "FAILED" : "all tests passed", failures,
	       failures == 1 ? "" : "s");
	return failures ? 1 : 0;
}
