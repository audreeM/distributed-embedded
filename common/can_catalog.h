/*
 * CAN message catalog for Lab 3
 *
 * Both Pi and STMs include this header to decode/encode frames
 * The functions only deal with the 8 data bytes; each node
 * copies those into and out of its own can_frame struct.
 *
 * Every message uses a standard 11-bit ID, always has 8 data bytes, and is
 * sent on both CAN A and CAN B. The first data byte always holds a 4-bit
 * sequence counter in its low bits. The top 4 bits are unused (send 0). Any
 * value bigger than one byte is stored low byte first, and bytes a message
 * doesn't use are sent as 0.
 *
 * How to send a message:
 *   Keep a separate sequence counter for each message ID you send. Pack the
 *   message with the current counter value, send those same 8 bytes on bus A
 *   and on bus B, then advance the counter once. Both copies share a counter
 *   value, and that's how the receiver tells they are the same message.
 *
 * How to receive a message:
 *   First note which bus it came in on, so you can track bus health. Then run
 *   it through can_seq_accept(); if that says no, it's the copy from the other
 *   bus (or an old leftover), so ignore it. Otherwise unpack it. If the unpack
 *   reports a problem with a control message, go to the error state and send
 *   a FAULT.
 */
#ifndef CAN_CATALOG_H
#define CAN_CATALOG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Message IDs
 *
 * CAN arbitration. the lowest ID wins when two nodes talk at once, so
 * these are listed from most to least important. CTRL comes first because it
 * carries the brake and throttle, which have to arrive within 2 ms. Fault and
 * heartbeat messages come next because they decide when the car stops safely.
 * Blinkers and force feedback are conveniences and come last.
 */
#define CAN_ID_CTRL  0x010 /* Pi to both STMs, on every wheel update (every 20 ms) */
#define CAN_ID_FAULT 0x020 /* any zone to everyone, when a zone fails or gets bad input */
#define CAN_ID_HB_N1 0x100 /* Pi heartbeat, every 20 ms */
#define CAN_ID_HB_N2 0x101 /* steering STM heartbeat, every 20 ms */
#define CAN_ID_HB_N3 0x102 /* drivetrain STM heartbeat, every 20 ms */
#define CAN_ID_BLINK 0x200 /* Pi to both STMs, whenever a blinker lamp turns on or off */
#define CAN_ID_FORCE 0x300 /* steering STM to Pi, every 50 ms */

#define CAN_DLC 8 /* every message is 8 bytes; anything else is treated as broken */

/* Zone numbers, used inside FAULT messages. */
#define CAN_ZONE_N1 1 /* cockpit (the Pi) */
#define CAN_ZONE_N2 2 /* steering STM */
#define CAN_ZONE_N3 3 /* drivetrain STM */

/* Which zone a heartbeat ID belongs to, or 0 if the ID isn't a heartbeat. */
static inline uint8_t can_hb_zone(uint16_t id)
{
	return (id >= CAN_ID_HB_N1 && id <= CAN_ID_HB_N3) ? (uint8_t)(id - CAN_ID_HB_N1 + 1) : 0;
}

/*
 * The sequence counter only needs 4 bits (0 to 15). The two copies of a
 * message arrive a few milliseconds apart, while it takes 16 messages (about
 * 320 ms) for the counter to come back around to the same value.
 */
#define CAN_SEQ_MASK 0x0F

// returns the uint8_t sequence counter from a can frame
static inline uint8_t can_seq(const uint8_t *d)
{
	return d[0] & CAN_SEQ_MASK;
}

// increments the sequence counter, overflowing to 0
static inline uint8_t can_seq_next(uint8_t seq)
{
	return (uint8_t)((seq + 1) & CAN_SEQ_MASK);
}

/*
 * What every unpack function returns. The output struct is filled in even
 * when the message is rejected, so you can still print what arrived.
 */
enum can_check {
	CAN_OK = 0,
	CAN_MALFORMED,    /* the message itself is broken (bad CRC, unknown value) */
	CAN_OUT_OF_RANGE, /* the message is intact but a value is outside its allowed range */
};

/*
 * CRC-8 using SAE-J1850
 * CAN already checks every frame for transmission errors, so this is
 * here to catch a node that builds the message wrong in software
 */
static inline uint8_t can_crc8(const uint8_t *buf, int len)
{
	uint8_t crc = 0xFF;

	for (int i = 0; i < len; i++) {
		crc ^= buf[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
		}
	}
	return crc ^ 0xFF;
}

/*
 * CTRL: the driving command from the wheel and pedals.
 *
 *   byte 0     sequence counter
 *   bytes 1-2  steering, -100 (full left) to +100 (full right), 0 is centered
 *   byte 3     throttle, 0 to 100 percent
 *   byte 4     brake, 0 to 100 percent. Anything above 0 means brake, and the
 *              brake always wins over the throttle.
 *   byte 5     which wheel buttons are pressed (for logging; the blinkers are
 *              controlled by BLINK messages, not by these bits)
 *   byte 6     unused
 *   byte 7     CRC of bytes 0 through 6 (only used by CTRL)
 */
#define CAN_BTN_LEFT  0x01
#define CAN_BTN_RIGHT 0x02
#define CAN_BTN_TEST  0x04
#define CAN_BTN_ALL   (CAN_BTN_LEFT | CAN_BTN_RIGHT | CAN_BTN_TEST)

struct can_ctrl {
	int16_t steer;
	uint8_t throttle;
	uint8_t brake;
	uint8_t buttons;
};

// packs d with the ctlr can frame
static inline void can_pack_ctrl(uint8_t d[CAN_DLC], uint8_t seq, const struct can_ctrl *c)
{
	d[0] = seq & CAN_SEQ_MASK;
	d[1] = (uint8_t)((uint16_t)c->steer & 0xFF);
	d[2] = (uint8_t)((uint16_t)c->steer >> 8);
	d[3] = c->throttle;
	d[4] = c->brake;
	d[5] = c->buttons;
	d[6] = 0;
	d[7] = can_crc8(d, 7);
}

/*
 * The CRC is checked first, if CRC fails, msg is ignored.
 * If CRC passes but value is out of range, the message was recieved
 * intact, so the sender is at fault.
 */
static inline enum can_check can_unpack_ctrl(const uint8_t d[CAN_DLC], struct can_ctrl *c)
{
	c->steer = (int16_t)(d[1] | (d[2] << 8));
	c->throttle = d[3];
	c->brake = d[4];
	c->buttons = d[5];
	if (can_crc8(d, 7) != d[7]) {
		return CAN_MALFORMED;
	}
	if (c->steer < -100 || c->steer > 100 || c->throttle > 100 || c->brake > 100 ||
	    (c->buttons & ~CAN_BTN_ALL) != 0 ||
	    /* the Pi never sends left and right at the same time */
	    (c->buttons & (CAN_BTN_LEFT | CAN_BTN_RIGHT)) == (CAN_BTN_LEFT | CAN_BTN_RIGHT)) {
		return CAN_OUT_OF_RANGE;
	}
	return CAN_OK;
}

/*
 * FAULT: one zone telling the others that something went wrong.
 *
 *   byte 0     sequence counter
 *   byte 1     which zone is reporting (CAN_ZONE_N1/N2/N3)
 *   byte 2     why (one of the CAN_FAULT_ reasons below)
 *
 * A zone sends this once when it fails or receives bad input, and never more
 * often than every 20 ms. If a zone is failing itself (for example from its
 * test button), it sends FAULT first and then goes quiet.
 *
 * All three zones share this one ID, and each keeps its own counter. So when
 * you receive a FAULT, unpack it first to see which zone sent it, then check
 * the counter against that zone's filter.
 */
#define CAN_FAULT_SELF_TEST 1 /* the zone's test button was pressed once */
#define CAN_FAULT_HB_LOST   2 /* another zone missed 3 heartbeats on both buses */
#define CAN_FAULT_BAD_VALUE 3 /* received a value outside its allowed range */
#define CAN_FAULT_MALFORMED 4 /* received a broken message */
#define CAN_FAULT_UDP_LOST  5 /* Pi only: no wheel data for 100 ms */
#define CAN_FAULT_MAX       5

struct can_fault {
	uint8_t zone;
	uint8_t reason;
};

static inline void can_pack_fault(uint8_t d[CAN_DLC], uint8_t seq, const struct can_fault *f)
{
	d[0] = seq & CAN_SEQ_MASK;
	d[1] = f->zone;
	d[2] = f->reason;
	d[3] = d[4] = d[5] = d[6] = d[7] = 0;
}

static inline enum can_check can_unpack_fault(const uint8_t d[CAN_DLC], struct can_fault *f)
{
	f->zone = d[1];
	f->reason = d[2];
	if (f->zone < CAN_ZONE_N1 || f->zone > CAN_ZONE_N3 ||
	    f->reason < 1 || f->reason > CAN_FAULT_MAX) {
		return CAN_MALFORMED;
	}
	return CAN_OK;
}

/*
 * Heartbeat (HB_N1, HB_N2, HB_N3): "I'm alive," sent every 20 ms.
 *
 *   byte 0     sequence counter
 *   byte 1     the sender's state: normal or error
 *   byte 2     the sender's bus health: bit 0 set means bus A is degraded,
 *              bit 1 set means bus B is degraded
 *
 * The ID tells you who sent it. A zone that has failed simply stops sending
 * heartbeats, so there's no "failed" value here; going quiet is the signal.
 */
#define CAN_STATE_ERROR  0 /* in the system-wide error state (also right after power-up) */
#define CAN_STATE_NORMAL 1

#define CAN_HEALTH_A_DEGRADED 0x01
#define CAN_HEALTH_B_DEGRADED 0x02

struct can_hb {
	uint8_t state;
	uint8_t health;
};

static inline void can_pack_hb(uint8_t d[CAN_DLC], uint8_t seq, const struct can_hb *h)
{
	d[0] = seq & CAN_SEQ_MASK;
	d[1] = h->state;
	d[2] = h->health;
	d[3] = d[4] = d[5] = d[6] = d[7] = 0;
}

static inline enum can_check can_unpack_hb(const uint8_t d[CAN_DLC], struct can_hb *h)
{
	h->state = d[1];
	h->health = d[2];
	if (h->state > CAN_STATE_NORMAL ||
	    (h->health & ~(CAN_HEALTH_A_DEGRADED | CAN_HEALTH_B_DEGRADED)) != 0) {
		return CAN_MALFORMED;
	}
	return CAN_OK;
}

/*
 * BLINK: which blinkers are active, and whether their lamps are on or off.
 *
 *   byte 0     sequence counter
 *   byte 1     mode: off, left, right or hazard
 *   byte 2     1 means the lamps turn on now, 0 means they turn off now
 *
 * The Pi keeps the blink timing and sends this every time the lamps should
 * switch. The two STMs don't run their own blink timers; they just set their
 * lamps the moment this message arrives. Since both STMs get the same message
 * at the same time, the front and rear lamps stay in step (the 1 ms
 * requirement). Separate timers on each board would slowly drift apart.
 *
 * What to do with each mode:
 *   off      all lamps off
 *   left     left lamps follow byte 2, right lamps off
 *   right    right lamps follow byte 2, left lamps off
 *   hazard   all four lamps follow byte 2
 */
#define CAN_BLINK_OFF    0
#define CAN_BLINK_LEFT   1
#define CAN_BLINK_RIGHT  2
#define CAN_BLINK_HAZARD 3

struct can_blink {
	uint8_t mode;
	uint8_t phase;
};

static inline void can_pack_blink(uint8_t d[CAN_DLC], uint8_t seq, const struct can_blink *b)
{
	d[0] = seq & CAN_SEQ_MASK;
	d[1] = b->mode;
	d[2] = b->phase;
	d[3] = d[4] = d[5] = d[6] = d[7] = 0;
}

static inline enum can_check can_unpack_blink(const uint8_t d[CAN_DLC], struct can_blink *b)
{
	b->mode = d[1];
	b->phase = d[2];
	return (b->mode <= CAN_BLINK_HAZARD && b->phase <= 1) ? CAN_OK : CAN_MALFORMED;
}

/*
 * FORCE: steering feedback for the wheel.
 *
 *   byte 0     sequence counter
 *   byte 1     force, -100 to +100
 *
 * This uses the same encoding as the force byte the proxy expects over UDP,
 * so the Pi can pass it straight through. In Lab 3 the steering STM just
 * sends a counting dummy value here.
 */
struct can_force {
	int8_t force;
};

static inline void can_pack_force(uint8_t d[CAN_DLC], uint8_t seq, const struct can_force *f)
{
	d[0] = seq & CAN_SEQ_MASK;
	d[1] = (uint8_t)f->force;
	d[2] = d[3] = d[4] = d[5] = d[6] = d[7] = 0;
}

static inline enum can_check can_unpack_force(const uint8_t d[CAN_DLC], struct can_force *f)
{
	f->force = (int8_t)d[1];
	return (f->force >= -100 && f->force <= 100) ? CAN_OK : CAN_OUT_OF_RANGE;
}

/*
 * Duplicate filter.
 *
 * Every message arrives twice, once per bus, and we only want to act on it
 * once. Keep one of these filters for each message you receive from each
 * sender, starting zeroed.
 *
 * The filter compares the incoming counter with the last one it accepted:
 *   - Same number: it's the second copy of a message already handled. Ignore.
 *   - 1 to 7 ahead: it's a new message (a few lost ones in between is fine).
 *   - Further than that, i.e. behind: it's an old message that got stuck in a
 *     controller while a bus wire was unplugged and came out late when the
 *     wire went back in. Ignore it, so an old brake or throttle value never
 *     gets applied.
 *
 * If nothing has been accepted from a sender for CAN_SEQ_RESYNC_MS, the next
 * message is accepted no matter its number. That handles a node that rebooted
 * (its counter starts over at 0) and messages that only come now and then,
 * like BLINK. 100 ms is comfortably longer than the gap between two copies of
 * a message, and shorter than the 160 ms the counter needs to get 8 steps
 * ahead at one message every 20 ms.
 */
#define CAN_SEQ_RESYNC_MS 100

struct can_seq_filter {
	uint8_t last;  /* counter value of the last accepted message */
	bool valid;    /* false until the first message arrives */
	uint32_t t_ms; /* when that message was accepted */
};

/* Returns true if this is a new message you should act on. */
static inline bool can_seq_accept(struct can_seq_filter *f, uint8_t seq, uint32_t now_ms)
{
	if (f->valid && (uint32_t)(now_ms - f->t_ms) <= CAN_SEQ_RESYNC_MS) {
		uint8_t ahead = (uint8_t)((seq - f->last) & CAN_SEQ_MASK);

		if (ahead == 0 || ahead >= 8) {
			return false;
		}
	}
	f->last = seq & CAN_SEQ_MASK;
	f->t_ms = now_ms;
	f->valid = true;
	return true;
}

#endif /* CAN_CATALOG_H */
