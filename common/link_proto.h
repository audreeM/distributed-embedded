/*
 * Pi <-> STM32 serial link protocol (Lab 2 stand-in for the CAN catalog).
 *
 * Shared, header-only, and free of OS dependencies so the exact same framing,
 * CRC and parser run on the Raspberry Pi (Linux, gcc) and the STM32 (Zephyr).
 * See LINK_PROTOCOL.md for the rationale.
 *
 * Frame:  A5 5A | TYPE | LEN | SEQ | PAYLOAD[LEN] | CRC16 lo | CRC16 hi
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD.
 * Multi-byte payload fields are little-endian.
 *
 * Every payload is exactly 8 bytes, the CAN 2.0A maximum, so each frame type
 * maps 1:1 onto a CAN message later (TYPE ~ CAN ID, LEN ~ DLC). Unused bytes
 * are reserved and must be 0.
 */
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LP_SYNC0 0xA5
#define LP_SYNC1 0x5A

#define LP_TYPE_CMD      0x10 /* Pi -> STM32: drive command */
#define LP_TYPE_STATUS   0x20 /* STM32 -> Pi: heartbeat, state + faults */
#define LP_TYPE_CURRENTS 0x21 /* STM32 -> Pi: current sensors, same 20 ms tick */

#define LP_PAYLOAD_LEN 8 /* every TYPE */
#define LP_CMD_RSVD    4 /* reserved bytes at the end of a CMD payload */

#define LP_HDR_LEN     5 /* sync0 sync1 type len seq */
#define LP_CRC_LEN     2
#define LP_MAX_PAYLOAD LP_PAYLOAD_LEN
#define LP_FRAME_MAX   (LP_HDR_LEN + LP_MAX_PAYLOAD + LP_CRC_LEN) /* 15 bytes */

/* CMD.buttons */
#define LP_BTN_LEFT      0x01
#define LP_BTN_RIGHT     0x02
#define LP_BTN_TEST      0x04
#define LP_BTN_VALID_MSK 0x07

/* STATUS.state */
#define LP_STATE_ERROR  0
#define LP_STATE_NORMAL 1

/* STATUS.faults - any bit set means the zone is in the error state */
#define LP_FAULT_POWERUP   0x01 /* no valid command since boot */
#define LP_FAULT_LINK_LOST 0x02 /* no valid command for LINK_TIMEOUT_MS */
#define LP_FAULT_BAD_CMD   0x04 /* CRC-valid command with out-of-range field */
#define LP_FAULT_SELF_TEST 0x08 /* self-test button single press */

struct lp_cmd {
	uint8_t throttle; /* 0..100 % */
	uint8_t brake;    /* 0..100 % */
	int8_t steer;     /* -100 full left .. +100 full right */
	uint8_t buttons;  /* LP_BTN_* */
	uint8_t reserved[LP_CMD_RSVD]; /* must be 0 */
};

struct lp_status {
	uint8_t state;     /* LP_STATE_* */
	uint8_t faults;    /* LP_FAULT_* */
	uint8_t last_seq;  /* SEQ of the last accepted command */
	uint8_t crc_err;   /* malformed frames dropped (saturates at 255) */
	uint8_t range_err; /* out-of-range commands rejected (saturates) */
};

struct lp_currents {
	uint16_t i_motor_l_ma;
	uint16_t i_motor_r_ma;
	uint16_t i_servo_ma;
};

struct lp_frame {
	uint8_t type;
	uint8_t len;
	uint8_t seq;
	uint8_t payload[LP_MAX_PAYLOAD];
};

/* ---------------------------------------------------------------- CRC */

/*
 * Bitwise CRC-16/CCITT-FALSE. A CRC catches every 1- and 2-bit error and
 * every burst of up to 16 bits, which a simple checksum does not. It is slower
 * than a lookup table, but ~8 shifts per byte is nothing at 115200 baud and
 * costs no flash.
 */
static inline uint16_t lp_crc16_update(uint16_t crc, uint8_t b)
{
	crc ^= (uint16_t)b << 8;
	for (int i = 0; i < 8; i++) {
		crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return crc;
}

static inline uint16_t lp_crc16(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc = lp_crc16_update(crc, buf[i]);
	}
	return crc;
}

/* ------------------------------------------------------------ encoding */

/* Returns the number of bytes written to out (at least LP_FRAME_MAX long). */
static inline size_t lp_encode(uint8_t *out, uint8_t type, uint8_t seq,
			       const uint8_t *payload, uint8_t len)
{
	size_t n = 0;

	out[n++] = LP_SYNC0;
	out[n++] = LP_SYNC1;
	out[n++] = type;
	out[n++] = len;
	out[n++] = seq;
	for (uint8_t i = 0; i < len; i++) {
		out[n++] = payload[i];
	}
	uint16_t crc = lp_crc16(&out[2], n - 2);

	out[n++] = (uint8_t)(crc & 0xFF);
	out[n++] = (uint8_t)(crc >> 8);
	return n;
}

static inline size_t lp_encode_cmd(uint8_t *out, uint8_t seq, const struct lp_cmd *c)
{
	uint8_t p[LP_PAYLOAD_LEN] = {
		c->throttle, c->brake, (uint8_t)c->steer, c->buttons,
		c->reserved[0], c->reserved[1], c->reserved[2], c->reserved[3],
	};

	return lp_encode(out, LP_TYPE_CMD, seq, p, LP_PAYLOAD_LEN);
}

static inline void lp_decode_cmd(const struct lp_frame *f, struct lp_cmd *c)
{
	c->throttle = f->payload[0];
	c->brake = f->payload[1];
	c->steer = (int8_t)f->payload[2];
	c->buttons = f->payload[3];
	for (int i = 0; i < LP_CMD_RSVD; i++) {
		c->reserved[i] = f->payload[4 + i];
	}
}

/*
 * Out-of-range check, applied AFTER the CRC has passed. A frame that gets
 * here arrived intact, so a bad value means the sender is wrong, not the
 * wire -- the STM32 treats that as a fault (BAD_CMD), not as noise.
 */
static inline bool lp_cmd_in_range(const struct lp_cmd *c)
{
	return c->throttle <= 100 && c->brake <= 100 &&
	       c->steer >= -100 && c->steer <= 100 &&
	       (c->buttons & ~LP_BTN_VALID_MSK) == 0 &&
	       (c->buttons & (LP_BTN_LEFT | LP_BTN_RIGHT)) != (LP_BTN_LEFT | LP_BTN_RIGHT) &&
	       (c->reserved[0] | c->reserved[1] | c->reserved[2] | c->reserved[3]) == 0;
}

static inline size_t lp_encode_status(uint8_t *out, uint8_t seq, const struct lp_status *s)
{
	uint8_t p[LP_PAYLOAD_LEN] = {
		s->state, s->faults, s->last_seq, s->crc_err, s->range_err, 0, 0, 0,
	};

	return lp_encode(out, LP_TYPE_STATUS, seq, p, LP_PAYLOAD_LEN);
}

static inline void lp_decode_status(const struct lp_frame *f, struct lp_status *s)
{
	const uint8_t *p = f->payload;

	s->state = p[0];
	s->faults = p[1];
	s->last_seq = p[2];
	s->crc_err = p[3];
	s->range_err = p[4];
}

static inline size_t lp_encode_currents(uint8_t *out, uint8_t seq, const struct lp_currents *c)
{
	uint8_t p[LP_PAYLOAD_LEN] = {
		(uint8_t)c->i_motor_l_ma, (uint8_t)(c->i_motor_l_ma >> 8),
		(uint8_t)c->i_motor_r_ma, (uint8_t)(c->i_motor_r_ma >> 8),
		(uint8_t)c->i_servo_ma, (uint8_t)(c->i_servo_ma >> 8),
		0, 0,
	};

	return lp_encode(out, LP_TYPE_CURRENTS, seq, p, LP_PAYLOAD_LEN);
}

static inline void lp_decode_currents(const struct lp_frame *f, struct lp_currents *c)
{
	const uint8_t *p = f->payload;

	c->i_motor_l_ma = (uint16_t)(p[0] | (p[1] << 8));
	c->i_motor_r_ma = (uint16_t)(p[2] | (p[3] << 8));
	c->i_servo_ma = (uint16_t)(p[4] | (p[5] << 8));
}

/* -------------------------------------------------------------- parser */

enum lp_result {
	LP_NONE = 0, /* byte consumed, no frame yet */
	LP_OK,       /* complete, CRC-valid frame in parser->frame */
	LP_ERR_LEN,  /* unknown TYPE or LEN that does not match TYPE */
	LP_ERR_CRC,  /* complete frame with a bad CRC */
	LP_ERR_GAP,  /* partial frame abandoned after an inter-byte gap */
};

enum lp_pstate {
	LP_S_SYNC0, LP_S_SYNC1, LP_S_TYPE, LP_S_LEN, LP_S_SEQ, LP_S_PAYLOAD,
	LP_S_CRC_LO, LP_S_CRC_HI,
};

struct lp_parser {
	uint8_t state;
	uint8_t idx;
	uint8_t crc_lo;
	uint16_t crc;
	uint32_t last_t;
	uint32_t gap_ticks; /* same unit as the timestamps passed in */
	struct lp_frame frame;
};

/*
 * gap_ticks: if more than this elapses between two bytes of one frame, the
 * partial frame is abandoned. A truncated frame therefore cannot swallow the
 * start of the next one, as long as frames are spaced further apart than this.
 */
static inline void lp_parser_init(struct lp_parser *p, uint32_t gap_ticks)
{
	p->state = LP_S_SYNC0;
	p->idx = 0;
	p->crc = 0xFFFF;
	p->last_t = 0;
	p->gap_ticks = gap_ticks;
}

/* LEN every known TYPE must carry (all 8 now), or -1 for an unknown TYPE. */
static inline int lp_expected_len(uint8_t type)
{
	switch (type) {
	case LP_TYPE_CMD:
	case LP_TYPE_STATUS:
	case LP_TYPE_CURRENTS:
		return LP_PAYLOAD_LEN;
	default:
		return -1;
	}
}

/*
 * Feed one byte. now: free-running timestamp; wrap-around is fine.
 *
 * State machine, one state per field:
 *   SYNC0 -> SYNC1 -> TYPE -> LEN -> SEQ -> PAYLOAD... -> CRC_LO -> CRC_HI
 * Any mismatch drops back to hunting for SYNC0, so a receiver that starts
 * mid-stream or loses a byte resynchronises on the next A5 5A.
 */
static inline enum lp_result lp_parse_byte(struct lp_parser *p, uint8_t b, uint32_t now)
{
	enum lp_result pending = LP_NONE;

	/* Truncated-frame detection: if we are in the middle of a frame and
	 * the line went quiet for too long, the rest of this frame is never
	 * coming. Abandon it; this byte is then treated as a possible SYNC0.
	 * (Unsigned subtraction gives the right delta even across wrap-around.)
	 */
	if (p->state != LP_S_SYNC0 && (uint32_t)(now - p->last_t) > p->gap_ticks) {
		p->state = LP_S_SYNC0;
		pending = LP_ERR_GAP;
	}
	p->last_t = now;

	switch (p->state) {
	case LP_S_SYNC0:
		if (b == LP_SYNC0) {
			p->state = LP_S_SYNC1;
		}
		break;
	case LP_S_SYNC1:
		if (b == LP_SYNC1) {
			p->state = LP_S_TYPE;
		} else if (b != LP_SYNC0) {
			p->state = LP_S_SYNC0;
		} /* A5 A5 5A: the second A5 may be the real start, so stay here */
		break;
	case LP_S_TYPE:
		p->frame.type = b;
		p->crc = lp_crc16_update(0xFFFF, b); /* CRC covers TYPE..PAYLOAD */
		p->state = LP_S_LEN;
		break;
	case LP_S_LEN:
		/* TYPE must be known and LEN must be 8. Checking it here rejects
		 * garbage early and guarantees payload[] can never overflow.
		 */
		if (lp_expected_len(p->frame.type) != b) {
			p->state = (b == LP_SYNC0) ? LP_S_SYNC1 : LP_S_SYNC0;
			return LP_ERR_LEN;
		}
		p->frame.len = b;
		p->crc = lp_crc16_update(p->crc, b);
		p->state = LP_S_SEQ;
		break;
	case LP_S_SEQ:
		p->frame.seq = b;
		p->crc = lp_crc16_update(p->crc, b);
		p->idx = 0;
		p->state = (p->frame.len == 0) ? LP_S_CRC_LO : LP_S_PAYLOAD;
		break;
	case LP_S_PAYLOAD:
		p->frame.payload[p->idx++] = b;
		p->crc = lp_crc16_update(p->crc, b);
		if (p->idx == p->frame.len) {
			p->state = LP_S_CRC_LO;
		}
		break;
	case LP_S_CRC_LO:
		p->crc_lo = b;
		p->state = LP_S_CRC_HI;
		break;
	case LP_S_CRC_HI:
		p->state = LP_S_SYNC0;
		if ((uint16_t)(p->crc_lo | (b << 8)) == p->crc) {
			return LP_OK;
		}
		return LP_ERR_CRC;
	default:
		p->state = LP_S_SYNC0;
		break;
	}
	return pending;
}

#endif /* LINK_PROTO_H */
