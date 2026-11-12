/*
 * Raft CAN messages and log entry format for lab 5.
 *
 * Same rules as can_catalog.h: 11-bit IDs, 8 data bytes, sent on both buses,
 * byte 0 low 4 bits = seq counter, high 4 bits = flags, little endian.
 *
 * Each message has one ID per sender (base + node ID) like HB_N1..N3, because
 * two nodes sending the same ID with different data at the same time breaks
 * CAN arbitration.
 *
 *   ID           Name         From -> to           Bytes 1..7
 *   0x031-0x033  REF_Nx       leader -> all        term, cycle u16, global time us u32 (filled by time sync)
 *   0x041-0x043  REQ_VOTE_Nx  candidate -> all     term, last index u16, last term
 *   0x045-0x047  VOTE_Nx      voter -> candidate   term, candidate, granted
 *   0x051-0x053  APPEND_Nx    leader -> all        term, index u16, prev term, entry, entry term, commit back
 *   0x055-0x057  APPEND_ACK_Nx follower -> leader  term, success, match/hint u16
 *   0x061-0x063  EVENT_Nx     client -> leader     entry
 *   0x065-0x067  REDIRECT_Nx  follower -> client   term, leader
 *
 * APPEND has one entry per frame. There's no room for a 16-bit commit index,
 * so "commit back" is how far behind the index field the leader's commit is:
 * 0 = commit >= index, k = commit is index - k, 0xFF = more than 254 back
 * (follower doesn't update its commit). An empty APPEND (entry = NONE) is sent
 * every 100 ms. Its index is the leader's last index and prev term is that
 * entry's term.
 *
 * Terms are 1 byte, so they wrap after 255 elections. fine for this lab.
 */
#ifndef RAFT_MSG_H
#define RAFT_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "raft_config.h"

#define RAFT_ID_REF        0x030
#define RAFT_ID_REQ_VOTE   0x040
#define RAFT_ID_VOTE       0x044
#define RAFT_ID_APPEND     0x050
#define RAFT_ID_APPEND_ACK 0x054
#define RAFT_ID_EVENT      0x060
#define RAFT_ID_REDIRECT   0x064

/* flags, high 4 bits of byte 0 */
#define RAFT_SEQ_MASK     0x0F
#define RAFT_FLAG_RELAYED 0x10 /* the relay sets this when it forwards a frame */
#define RAFT_FLAG_PREVOTE 0x20 /* REQ_VOTE / VOTE: this is a pre-vote */

enum raft_msg {
	RAFT_MSG_NONE = 0,
	RAFT_MSG_REF,
	RAFT_MSG_REQ_VOTE,
	RAFT_MSG_VOTE,
	RAFT_MSG_APPEND,
	RAFT_MSG_APPEND_ACK,
	RAFT_MSG_EVENT,
	RAFT_MSG_REDIRECT,
};

/* which raft message this ID is and who sent it, RAFT_MSG_NONE if it's not raft */
static inline enum raft_msg raft_msg_from_id(uint16_t id, uint8_t *sender)
{
	static const struct {
		uint16_t base;
		enum raft_msg msg;
	} map[] = {
		{RAFT_ID_REF, RAFT_MSG_REF},           {RAFT_ID_REQ_VOTE, RAFT_MSG_REQ_VOTE},
		{RAFT_ID_VOTE, RAFT_MSG_VOTE},         {RAFT_ID_APPEND, RAFT_MSG_APPEND},
		{RAFT_ID_APPEND_ACK, RAFT_MSG_APPEND_ACK}, {RAFT_ID_EVENT, RAFT_MSG_EVENT},
		{RAFT_ID_REDIRECT, RAFT_MSG_REDIRECT},
	};

	for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
		if (id > map[i].base && id <= map[i].base + RAFT_N) {
			*sender = (uint8_t)(id - map[i].base);
			return map[i].msg;
		}
	}
	*sender = 0;
	return RAFT_MSG_NONE;
}

/* log entries */

/*
 * Each entry is 1 byte (log stores term + entry = 2 bytes).
 *
 *   bits 7-6  kind
 *   00        0x00 NONE (empty APPEND, never stored), 0x01 COCKPIT_OK,
 *             0x02 COCKPIT_ERROR, 0x05-0x07 LEADER_Nx (new leader's first entry)
 *   01        MEMBER node: bits 2-1 node, bit 0 up        NODE_3_DOWN      = 0x46
 *   10        MEMBER link: bits 3-2 node, bit 1 bus, bit 0 up   CANLINK_1_B_DOWN = 0x86
 *   11        EVENT: bits 5-4 node, bits 3-0 Lab 1 error code   N2 self-test     = 0xE1
 */
#define RAFT_E_NONE          0x00
#define RAFT_E_COCKPIT_OK    0x01
#define RAFT_E_COCKPIT_ERROR 0x02

#define RAFT_BUS_A 0
#define RAFT_BUS_B 1

static inline uint8_t raft_e_leader(uint8_t node) { return (uint8_t)(0x04 | (node & 0x3)); }

static inline uint8_t raft_e_node(uint8_t node, bool up)
{
	return (uint8_t)(0x40 | ((node & 0x3) << 1) | (up ? 1 : 0));
}

static inline uint8_t raft_e_link(uint8_t node, uint8_t bus, bool up)
{
	return (uint8_t)(0x80 | ((node & 0x3) << 2) | ((bus & 1) << 1) | (up ? 1 : 0));
}

static inline uint8_t raft_e_event(uint8_t node, uint8_t code)
{
	return (uint8_t)(0xC0 | ((node & 0x3) << 4) | (code & 0xF));
}

enum raft_ekind {
	RAFT_EK_INVALID = 0,
	RAFT_EK_NONE,
	RAFT_EK_COCKPIT, /* up = cockpit OK */
	RAFT_EK_LEADER,
	RAFT_EK_NODE,    /* MEMBER */
	RAFT_EK_LINK,    /* MEMBER */
	RAFT_EK_EVENT,
};

struct raft_edec {
	enum raft_ekind kind;
	uint8_t node; /* 1..3 */
	uint8_t bus;  /* RAFT_BUS_A / RAFT_BUS_B */
	uint8_t code; /* EVENT only */
	bool up;
};

static inline struct raft_edec raft_entry_decode(uint8_t e)
{
	struct raft_edec d = {RAFT_EK_INVALID, 0, 0, 0, false};

	switch (e >> 6) {
	case 0:
		if (e == RAFT_E_NONE) {
			d.kind = RAFT_EK_NONE;
		} else if (e == RAFT_E_COCKPIT_OK || e == RAFT_E_COCKPIT_ERROR) {
			d.kind = RAFT_EK_COCKPIT;
			d.node = RAFT_NODE_COCKPIT;
			d.up = e == RAFT_E_COCKPIT_OK;
		} else if ((e & 0xFC) == 0x04 && (e & 0x3) != 0) {
			d.kind = RAFT_EK_LEADER;
			d.node = e & 0x3;
		}
		break;
	case 1:
		if ((e & 0x38) == 0) { /* bits 5-3 unused */
			d.kind = RAFT_EK_NODE;
			d.node = (e >> 1) & 0x3;
			d.up = e & 1;
		}
		break;
	case 2:
		if ((e & 0x30) == 0) { /* bits 5-4 unused */
			d.kind = RAFT_EK_LINK;
			d.node = (e >> 2) & 0x3;
			d.bus = (e >> 1) & 1;
			d.up = e & 1;
		}
		break;
	default:
		d.kind = RAFT_EK_EVENT;
		d.node = (e >> 4) & 0x3;
		d.code = e & 0xF;
		break;
	}
	if (d.kind != RAFT_EK_NONE && d.kind != RAFT_EK_INVALID &&
	    (d.node < 1 || d.node > RAFT_N)) {
		d.kind = RAFT_EK_INVALID;
	}
	return d;
}

/* entries a client is allowed to send (not NONE, LEADER or garbage) */
static inline bool raft_entry_submittable(uint8_t e)
{
	enum raft_ekind k = raft_entry_decode(e).kind;

	return k == RAFT_EK_COCKPIT || k == RAFT_EK_NODE || k == RAFT_EK_LINK ||
	       k == RAFT_EK_EVENT;
}

/* printable name like CANLINK_1_A_DOWN */
static inline const char *raft_entry_str(uint8_t e, char *buf, size_t len)
{
	struct raft_edec d = raft_entry_decode(e);

	switch (d.kind) {
	case RAFT_EK_NONE:    snprintf(buf, len, "NONE"); break;
	case RAFT_EK_COCKPIT: snprintf(buf, len, "COCKPIT_%s", d.up ? "OK" : "ERROR"); break;
	case RAFT_EK_LEADER:  snprintf(buf, len, "LEADER_N%u", d.node); break;
	case RAFT_EK_NODE:    snprintf(buf, len, "NODE_%u_%s", d.node, d.up ? "UP" : "DOWN"); break;
	case RAFT_EK_LINK:
		snprintf(buf, len, "CANLINK_%u_%c_%s", d.node, d.bus ? 'B' : 'A', d.up ? "UP" : "DOWN");
		break;
	case RAFT_EK_EVENT:   snprintf(buf, len, "EVENT_N%u_0x%02X", d.node, d.code); break;
	default:              snprintf(buf, len, "INVALID_0x%02X", e); break;
	}
	return buf;
}

/* packing / unpacking frames */

static inline void raft_put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t raft_get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static inline void raft_put32(uint8_t *p, uint32_t v)
{
	raft_put16(p, (uint16_t)v);
	raft_put16(p + 2, (uint16_t)(v >> 16));
}

static inline uint32_t raft_get32(const uint8_t *p)
{
	return (uint32_t)raft_get16(p) | ((uint32_t)raft_get16(p + 2) << 16);
}

static inline void raft_hdr(uint8_t d[8], uint8_t seq, uint8_t flags)
{
	for (int i = 0; i < 8; i++) {
		d[i] = 0;
	}
	d[0] = (uint8_t)((seq & RAFT_SEQ_MASK) | (flags & ~RAFT_SEQ_MASK));
}

static inline uint8_t raft_seq(const uint8_t d[8]) { return d[0] & RAFT_SEQ_MASK; }
static inline uint8_t raft_flags(const uint8_t d[8]) { return d[0] & ~RAFT_SEQ_MASK; }

/* REF is both the raft heartbeat and the time sync reference */
struct raft_ref {
	uint8_t term;
	uint16_t cycle;   /* +1 every REF */
	uint32_t time_us; /* leader's global time (low 32 bits), time sync fills this in */
};

static inline void raft_pack_ref(uint8_t d[8], uint8_t seq, const struct raft_ref *r)
{
	raft_hdr(d, seq, 0);
	d[1] = r->term;
	raft_put16(&d[2], r->cycle);
	raft_put32(&d[4], r->time_us);
}

static inline void raft_unpack_ref(const uint8_t d[8], struct raft_ref *r)
{
	r->term = d[1];
	r->cycle = raft_get16(&d[2]);
	r->time_us = raft_get32(&d[4]);
}

struct raft_req_vote {
	uint8_t term;
	uint16_t last_index;
	uint8_t last_term;
	bool prevote;
};

static inline void raft_pack_req_vote(uint8_t d[8], uint8_t seq, const struct raft_req_vote *v)
{
	raft_hdr(d, seq, v->prevote ? RAFT_FLAG_PREVOTE : 0);
	d[1] = v->term;
	raft_put16(&d[2], v->last_index);
	d[4] = v->last_term;
}

static inline void raft_unpack_req_vote(const uint8_t d[8], struct raft_req_vote *v)
{
	v->prevote = (d[0] & RAFT_FLAG_PREVOTE) != 0;
	v->term = d[1];
	v->last_index = raft_get16(&d[2]);
	v->last_term = d[4];
}

struct raft_vote {
	uint8_t term;
	uint8_t candidate;
	bool granted;
	bool prevote;
};

static inline void raft_pack_vote(uint8_t d[8], uint8_t seq, const struct raft_vote *v)
{
	raft_hdr(d, seq, v->prevote ? RAFT_FLAG_PREVOTE : 0);
	d[1] = v->term;
	d[2] = v->candidate;
	d[3] = v->granted ? 1 : 0;
}

static inline void raft_unpack_vote(const uint8_t d[8], struct raft_vote *v)
{
	v->prevote = (d[0] & RAFT_FLAG_PREVOTE) != 0;
	v->term = d[1];
	v->candidate = d[2];
	v->granted = d[3] != 0;
}

#define RAFT_COMMIT_UNKNOWN 0xFF

struct raft_append {
	uint8_t term;       /* leader's current term */
	uint16_t index;     /* index of this entry, or leader's last index if empty */
	uint8_t prev_term;  /* term of the entry before this one (or at index if empty) */
	uint8_t entry;      /* RAFT_E_NONE = empty */
	uint8_t entry_term; /* term this entry was added in */
	uint8_t commit_back;
};

static inline void raft_pack_append(uint8_t d[8], uint8_t seq, const struct raft_append *a)
{
	raft_hdr(d, seq, 0);
	d[1] = a->term;
	raft_put16(&d[2], a->index);
	d[4] = a->prev_term;
	d[5] = a->entry;
	d[6] = a->entry_term;
	d[7] = a->commit_back;
}

static inline void raft_unpack_append(const uint8_t d[8], struct raft_append *a)
{
	a->term = d[1];
	a->index = raft_get16(&d[2]);
	a->prev_term = d[4];
	a->entry = d[5];
	a->entry_term = d[6];
	a->commit_back = d[7];
}

struct raft_ack {
	uint8_t term;
	bool success;
	uint16_t match; /* success: last index we match. fail: where the leader should back up to */
};

static inline void raft_pack_ack(uint8_t d[8], uint8_t seq, const struct raft_ack *a)
{
	raft_hdr(d, seq, 0);
	d[1] = a->term;
	d[2] = a->success ? 1 : 0;
	raft_put16(&d[3], a->match);
}

static inline void raft_unpack_ack(const uint8_t d[8], struct raft_ack *a)
{
	a->term = d[1];
	a->success = d[2] != 0;
	a->match = raft_get16(&d[3]);
}

static inline void raft_pack_event(uint8_t d[8], uint8_t seq, uint8_t entry)
{
	raft_hdr(d, seq, 0);
	d[1] = entry;
}

static inline uint8_t raft_unpack_event(const uint8_t d[8]) { return d[1]; }

struct raft_redirect {
	uint8_t term;
	uint8_t leader;
};

static inline void raft_pack_redirect(uint8_t d[8], uint8_t seq, const struct raft_redirect *r)
{
	raft_hdr(d, seq, 0);
	d[1] = r->term;
	d[2] = r->leader;
}

static inline void raft_unpack_redirect(const uint8_t d[8], struct raft_redirect *r)
{
	r->term = d[1];
	r->leader = d[2];
}

#endif /* RAFT_MSG_H */
