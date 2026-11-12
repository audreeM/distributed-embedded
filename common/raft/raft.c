/* Raft for lab 5. See raft.h for how to use it. */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "raft.h"

/* separate seq counter for each kind of frame we send */
enum { TXS_REF, TXS_REQ_VOTE, TXS_VOTE, TXS_APPEND, TXS_ACK, TXS_EVENT, TXS_REDIRECT };

#define MS(x) ((uint64_t)(x) * 1000u)

/* ======================================================== helpers */

static uint64_t get_time_us(const struct raft_node *n) { return n->port->now_us(n->port->ctx); }

/*
 * printf-style debug print. Every line starts with the node, its term and its
 * role, e.g. "[raft N2 t1 follower] append #3 CANLINK_2_A_DOWN". Does nothing
 * if RAFT_DEBUG is 0 or the node didn't give us a log function.
 */
static void debug_print(const struct raft_node *n, const char *fmt, ...)
{
#if RAFT_DEBUG
	char buf[160];
	int k;
	va_list ap;

	if (n->port->log == NULL) {
		return;
	}
	k = snprintf(buf, sizeof(buf), "[raft N%u t%u %s] ", n->id, n->term,
		     raft_role_str(n->role));
	va_start(ap, fmt);
	vsnprintf(buf + k, sizeof(buf) - (size_t)k, fmt, ap);
	va_end(ap);
	n->port->log(n->port->ctx, buf);
#else
	(void)n;
	(void)fmt;
#endif
}

/* Returns the next 4-bit seq number for this kind of frame (0..15, wraps). */
static uint8_t next_seq(struct raft_node *n, int which)
{
	uint8_t s = n->tx_seq[which];

	n->tx_seq[which] = (uint8_t)((s + 1) & RAFT_SEQ_MASK);
	return s;
}

/* Sends a frame on both buses. The CAN ID is the message's base ID + our node ID. */
static void send_frame(struct raft_node *n, uint16_t base, const uint8_t d[8])
{
	n->port->send(n->port->ctx, (uint16_t)(base + n->id), d);
}

/* Random number generator (xorshift). The same seed always gives the same numbers. */
static uint32_t random_u32(struct raft_node *n)
{
	uint32_t x = n->rng;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	n->rng = x;
	return x;
}

/*
 * How long a follower waits without hearing the leader before it tries to
 * become leader: missed_heartbeats * heartbeat_ms + random 0..jitter.
 * With the defaults that's 60-99 ms. The random part means two followers
 * usually don't time out at the same moment and split the vote.
 */
static uint64_t election_timeout(struct raft_node *n)
{
	return MS(RAFT_MISSED_HEARTBEATS * RAFT_HEARTBEAT_MS +
		  random_u32(n) % (RAFT_ELECTION_JITTER_MS + 1));
}

static bool is_valid_node(uint8_t id) { return id >= 1 && id <= RAFT_N; }

/* Number of 1 bits. votes is a bitmask, so this is how many votes we have. */
static int count_bits(uint8_t v)
{
	int c = 0;

	for (; v; v &= (uint8_t)(v - 1)) {
		c++;
	}
	return c;
}

/* Term of the newest entry in our log (0 if the log is empty). */
static uint8_t last_log_term(const struct raft_node *n) { return n->log[n->last_index].term; }

/* Tells the node to save term + vote (the Pi writes a file, the STM32s don't). */
static void save_term_and_vote(struct raft_node *n)
{
	if (n->port->save_state) {
		n->port->save_state(n->port->ctx, n->term, n->voted_for);
	}
}

/*
 * In default leader mode, followers don't run their election timer until
 * they've heard from the leader at least once. Otherwise the STM32s would
 * hold an election while the Pi is still booting.
 */
static bool elections_allowed(const struct raft_node *n)
{
	return n->heard_leader || !is_valid_node(n->opts.default_leader);
}

/*
 * True if we heard from a leader in the last missed_heartbeats * heartbeat_ms
 * (60 ms). While this is true we refuse to vote for anyone else: the leader
 * is alive, so whoever is asking probably just lost a wire.
 */
static bool leader_recently_heard(const struct raft_node *n, uint64_t t)
{
	if (n->role == RAFT_LEADER) {
		return true;
	}
	return n->leader != 0 &&
	       t - n->last_leader_us < MS(RAFT_MISSED_HEARTBEATS * RAFT_HEARTBEAT_MS);
}

const char *raft_role_str(enum raft_role r)
{
	switch (r) {
	case RAFT_FOLLOWER:     return "follower";
	case RAFT_PRECANDIDATE: return "precandidate";
	case RAFT_CANDIDATE:    return "candidate";
	case RAFT_LEADER:       return "leader";
	default:                return "?";
	}
}

/* ======================================================== log */

/*
 * Leader only: adds an entry to the end of our log with the current term.
 * Also applies it to pending_net so we can tell if later reports are
 * duplicates. Returns the new entry's index, or 0 if the log is full.
 */
static uint16_t log_append(struct raft_node *n, uint8_t entry)
{
	char s[24];

	if (n->last_index >= RAFT_LOG_SIZE) {
		debug_print(n, "log full, dropping %s", raft_entry_str(entry, s, sizeof(s)));
		return 0;
	}
	n->last_index++;
	n->log[n->last_index].term = n->term;
	n->log[n->last_index].entry = entry;
	sm_apply(&n->pending_net, entry);
	return n->last_index;
}

/*
 * pending_net = committed network + every entry after it that isn't
 * committed yet. Rebuilt when we become leader, since our log may have
 * changed while we were a follower.
 */
static void rebuild_pending_net(struct raft_node *n)
{
	n->pending_net = n->net;
	for (uint16_t i = n->applied_index + 1; i <= n->last_index; i++) {
		sm_apply(&n->pending_net, n->log[i].entry);
	}
}

/*
 * Applies every entry that's committed but not applied yet, in order. This
 * is the only place the network graph (n->net, what getNetwork returns)
 * changes. Also tells the node through entry_committed so the Pi can print it.
 */
static void apply_committed_entries(struct raft_node *n)
{
	char s[24];

	while (n->applied_index < n->commit_index) {
		uint16_t i = ++n->applied_index;
		bool replayed = i <= n->replay_until;

		sm_apply(&n->net, n->log[i].entry);
		debug_print(n, "commit #%u %s%s", i, raft_entry_str(n->log[i].entry, s, sizeof(s)),
			    replayed ? " [replayed]" : "");
		if (n->port->entry_committed) {
			n->port->entry_committed(n->port->ctx, i, n->log[i].term, n->log[i].entry, replayed);
		}
	}
}

/* ======================================================== changing roles */

/*
 * Go back to being a follower of `leader` (0 if we don't know who it is yet).
 * If `term` is newer than ours we move to it and forget who we voted for,
 * since that vote was for the old term. Restarts the election timer.
 */
static void become_follower(struct raft_node *n, uint8_t term, uint8_t leader, uint64_t t)
{
	if (term > n->term) {
		n->term = term;
		n->voted_for = 0;
		save_term_and_vote(n);
	}
	if (n->role != RAFT_FOLLOWER || n->leader != leader) {
		n->role = RAFT_FOLLOWER;
		n->leader = leader;
		if (leader) {
			debug_print(n, "following N%u", leader);
		}
	}
	n->deadline_us = t + election_timeout(n);
}

/*
 * Leader: sends the REF frame (term + cycle counter). This is the heartbeat
 * that tells followers we're alive. Time sync fills in bytes 4-7 with the
 * global time through the fill_ref hook right before it goes out.
 */
static void send_heartbeat(struct raft_node *n)
{
	struct raft_ref r = {.term = n->term, .cycle = n->ref_cycle++, .time_us = 0};
	uint8_t d[8];

	raft_pack_ref(d, next_seq(n, TXS_REF), &r);
	if (n->port->fill_ref) {
		n->port->fill_ref(n->port->ctx, d);
	}
	send_frame(n, RAFT_ID_REF, d);
}

/*
 * We won the election (or we're the default leader at boot). Resets what we
 * know about each follower, adds a LEADER_Nx entry, and sends a heartbeat
 * right away so the others stop their election timers.
 */
static void become_leader(struct raft_node *n, uint64_t t)
{
	n->role = RAFT_LEADER;
	n->leader = n->id;
	n->heard_leader = true;
	n->quorum_seen = false;
	for (uint8_t f = 1; f <= RAFT_N; f++) {
		/* assume every follower is up to date until it says otherwise */
		n->next_index[f] = (uint16_t)(n->last_index + 1);
		n->match_index[f] = 0;
		/* treat everyone as unreachable until they reply, so we don't
		 * waste frames sending the log to a node that's off */
		n->last_ack_us[f] = t - MS(RAFT_QUORUM_LOST_MS);
	}
	rebuild_pending_net(n);
	debug_print(n, "became leader");
	/* every new leader adds a LEADER entry. raft needs an entry from the
	 * current term before older entries can commit, and it logs the change */
	log_append(n, raft_e_leader(n->id));
	send_heartbeat(n);
	n->next_ref_us = t + MS(RAFT_HEARTBEAT_MS);
	n->next_append_us = t;
	n->next_idle_us = t;
}

/*
 * Sends REQ_VOTE: "vote for me in `term`". Includes how long our log is and
 * the term of our last entry so others only vote for us if our log is at
 * least as up to date as theirs. prevote = only asking, not a real vote.
 */
static void send_vote_request(struct raft_node *n, uint8_t term, bool prevote)
{
	struct raft_req_vote v = {
		.term = term, .last_index = n->last_index, .last_term = last_log_term(n), .prevote = prevote,
	};
	uint8_t d[8];

	raft_pack_req_vote(d, next_seq(n, TXS_REQ_VOTE), &v);
	send_frame(n, RAFT_ID_REQ_VOTE, d);
}

/*
 * Real election: move to the next term, vote for ourselves and ask the
 * others. Only happens after a pre-vote said the others would agree.
 */
static void start_election(struct raft_node *n, uint64_t t)
{
	n->term++;
	n->voted_for = n->id;
	save_term_and_vote(n);
	n->role = RAFT_CANDIDATE;
	n->leader = 0;
	n->votes = (uint8_t)(1u << n->id);
	n->deadline_us = t + election_timeout(n);
	debug_print(n, "election for term %u", n->term);
	send_vote_request(n, n->term, false);
	if (count_bits(n->votes) >= RAFT_MAJORITY) {
		become_leader(n, t); /* only if N = 1 */
	}
}

/*
 * Our election timer ran out. Before starting a real election (which bumps
 * the term for everyone), ask "would you vote for me?". Our term doesn't
 * change. If a majority says yes we start the real election, otherwise we
 * try again after another timeout.
 */
static void start_prevote(struct raft_node *n, uint64_t t)
{
	n->role = RAFT_PRECANDIDATE;
	n->leader = 0;
	n->votes = (uint8_t)(1u << n->id);
	n->deadline_us = t + election_timeout(n);
	debug_print(n, "leader timeout, pre-vote for term %u", n->term + 1);
	send_vote_request(n, (uint8_t)(n->term + 1), true);
	if (count_bits(n->votes) >= RAFT_MAJORITY) {
		start_election(n, t);
	}
}

/*
 * Called when a REF or APPEND comes in from node s claiming to be leader of
 * `term`. If the term is current, s is our leader: follow it and restart our
 * election timer. Returns false if s is an old leader we should ignore.
 */
static bool heard_from_leader(struct raft_node *n, uint8_t s, uint8_t term, uint64_t t)
{
	if (term < n->term) {
		return false;
	}
	if (term == n->term && n->role == RAFT_LEADER) {
		debug_print(n, "N%u also claims term %u, ignoring", s, term);
		return false;
	}
	if (term > n->term || n->role != RAFT_FOLLOWER || n->leader != s) {
		become_follower(n, term, s, t);
	}
	n->heard_leader = true;
	n->last_leader_us = t;
	n->deadline_us = t + election_timeout(n);
	return true;
}

/* ======================================================== leader: sending the log */

/*
 * Sends one APPEND frame. index = which log entry to send, or 0 for an empty
 * APPEND (no entry, just tells followers our last index and commit index).
 *
 * prev_term is the term of the entry before this one. The follower checks
 * that it has the same entry there, which is how Raft makes sure the logs
 * match all the way back.
 */
static void send_append_entries(struct raft_node *n, uint16_t index)
{
	struct raft_append a = {.term = n->term};
	uint8_t d[8];

	if (index == 0) {
		a.index = n->last_index;
		a.prev_term = last_log_term(n);
		a.entry = RAFT_E_NONE;
	} else {
		a.index = index;
		a.prev_term = n->log[index - 1].term;
		a.entry = n->log[index].entry;
		a.entry_term = n->log[index].term;
	}
	/* commit index as "how far behind index", see raft_msg.h */
	if (n->commit_index >= a.index) {
		a.commit_back = 0;
	} else if (a.index - n->commit_index < RAFT_COMMIT_UNKNOWN) {
		a.commit_back = (uint8_t)(a.index - n->commit_index);
	} else {
		a.commit_back = RAFT_COMMIT_UNKNOWN;
	}
	raft_pack_append(d, next_seq(n, TXS_APPEND), &a);
	send_frame(n, RAFT_ID_APPEND, d);
}

/*
 * Leader: an entry is committed once a majority of nodes (us included) have
 * it. Walks back from the newest entry and stops at the first one a majority
 * has. If the commit index moved, sends an empty APPEND right away so the
 * followers commit too.
 */
static void update_commit_index(struct raft_node *n, uint64_t t)
{
	for (uint16_t i = n->last_index; i > n->commit_index; i--) {
		int count = 1; /* us */

		if (n->log[i].term != n->term) {
			break; /* raft rule: only count copies of entries from our own term */
		}
		for (uint8_t f = 1; f <= RAFT_N; f++) {
			if (f != n->id && n->match_index[f] >= i) {
				count++;
			}
		}
		if (count >= RAFT_MAJORITY) {
			n->commit_index = i;
			n->next_idle_us = t; /* tell followers right away instead of waiting 100 ms */
			break;
		}
	}
}

/*
 * Leader, every tick. If a follower is missing entries, send it the oldest
 * one it needs (one every 2 ms). Followers that haven't replied in 200 ms are
 * skipped so we don't flood the bus for a node that's off. If nobody needs
 * anything, send an empty APPEND every 100 ms.
 *
 * APPEND goes to everyone, so one frame can catch up two followers at once.
 */
static void send_log_to_followers(struct raft_node *n, uint64_t t)
{
	uint16_t oldest_needed = 0;

	for (uint8_t f = 1; f <= RAFT_N; f++) {
		if (f == n->id || n->next_index[f] > n->last_index ||
		    t - n->last_ack_us[f] >= MS(RAFT_FOLLOWER_ACTIVE_MS)) {
			continue;
		}
		if (oldest_needed == 0 || n->next_index[f] < oldest_needed) {
			oldest_needed = n->next_index[f];
		}
	}
	if (oldest_needed != 0 && t >= n->next_append_us) {
		send_append_entries(n, oldest_needed < 1 ? 1 : oldest_needed);
		n->next_append_us = t + MS(RAFT_APPEND_RETRY_MS);
		return;
	}
	if (t >= n->next_idle_us) {
		send_append_entries(n, 0);
		n->next_idle_us = t + MS(RAFT_APPEND_IDLE_MS);
	}
}

/*
 * Leader, every tick. If we had a majority before but now fewer than a
 * majority have replied in the last 300 ms, we're probably cut off, so step
 * down. The other side can then elect a leader and our LEADER LED goes off.
 */
static void check_majority_alive(struct raft_node *n, uint64_t t)
{
	int active = 1; /* us */

	for (uint8_t f = 1; f <= RAFT_N; f++) {
		if (f != n->id && t - n->last_ack_us[f] < MS(RAFT_QUORUM_LOST_MS)) {
			active++;
		}
	}
	if (active >= RAFT_MAJORITY) {
		n->quorum_seen = true;
	} else if (n->quorum_seen) {
		debug_print(n, "lost majority, stepping down");
		n->role = RAFT_FOLLOWER;
		n->leader = 0;
		n->deadline_us = t + election_timeout(n);
	}
}

/* ======================================================== handling received frames */

/* REF from node s: the leader's heartbeat. */
static void handle_heartbeat(struct raft_node *n, uint8_t s, const uint8_t d[8], uint64_t t)
{
	struct raft_ref r;

	raft_unpack_ref(d, &r);
	heard_from_leader(n, s, r.term, t);
	/* the time part of REF is handled by time sync, not here */
}

/*
 * REQ_VOTE from node s. We vote yes only if:
 *  - its term is newer than ours (or the same and we haven't voted for someone else)
 *  - its log is at least as up to date as ours, so a winner never loses committed entries
 *  - we haven't heard from a leader in the last 60 ms
 * Pre-votes get the same checks but change nothing on our side.
 */
static void handle_vote_request(struct raft_node *n, uint8_t s, const uint8_t d[8], uint64_t t)
{
	struct raft_req_vote rv;
	struct raft_vote v = {.candidate = s};
	uint8_t out[8];
	bool log_ok;

	raft_unpack_req_vote(d, &rv);
	log_ok = rv.last_term > last_log_term(n) ||
		 (rv.last_term == last_log_term(n) && rv.last_index >= n->last_index);

	if (rv.prevote) {
		v.prevote = true;
		v.term = rv.term;
		v.granted = rv.term > n->term && log_ok && !leader_recently_heard(n, t);
	} else {
		if (rv.term > n->term) {
			if (leader_recently_heard(n, t)) {
				debug_print(n, "ignoring vote request from N%u: leader alive", s);
				return;
			}
			become_follower(n, rv.term, 0, t);
		}
		v.term = n->term;
		if (rv.term == n->term && (n->voted_for == 0 || n->voted_for == s) && log_ok) {
			n->voted_for = s;
			save_term_and_vote(n);
			n->deadline_us = t + election_timeout(n);
			v.granted = true;
		}
	}
	debug_print(n, "%svote for N%u term %u: %s", v.prevote ? "pre-" : "", s, rv.term,
		    v.granted ? "yes" : "no");
	raft_pack_vote(out, next_seq(n, TXS_VOTE), &v);
	send_frame(n, RAFT_ID_VOTE, out);
}

/*
 * VOTE from node s, answering our REQ_VOTE. Counts yes votes. A majority of
 * pre-votes starts the real election, and a majority of real votes makes us
 * leader. A reply with a newer term means we're behind, so we go back to
 * being a follower.
 */
static void handle_vote_reply(struct raft_node *n, uint8_t s, const uint8_t d[8], uint64_t t)
{
	struct raft_vote v;

	raft_unpack_vote(d, &v);
	if (v.candidate != n->id) {
		return; /* answer to someone else's request */
	}
	if (v.prevote) {
		if (n->role == RAFT_PRECANDIDATE && v.term == (uint8_t)(n->term + 1) && v.granted) {
			n->votes |= (uint8_t)(1u << s);
			if (count_bits(n->votes) >= RAFT_MAJORITY) {
				start_election(n, t);
			}
		}
		return;
	}
	if (v.term > n->term) {
		become_follower(n, v.term, 0, t);
		return;
	}
	if (n->role == RAFT_CANDIDATE && v.term == n->term && v.granted) {
		n->votes |= (uint8_t)(1u << s);
		if (count_bits(n->votes) >= RAFT_MAJORITY) {
			become_leader(n, t);
		}
	}
}

/*
 * Follower: sends APPEND_ACK. success = we stored it, match = our last entry
 * that matches the leader. On failure, match tells the leader where to back
 * up to.
 */
static void send_append_reply(struct raft_node *n, bool success, uint16_t match)
{
	struct raft_ack a = {.term = n->term, .success = success, .match = match};
	uint8_t d[8];

	raft_pack_ack(d, next_seq(n, TXS_ACK), &a);
	send_frame(n, RAFT_ID_APPEND_ACK, d);
}

/*
 * APPEND from leader s. Steps:
 *  1. Old term? Reply no, so the old leader finds out it's been replaced.
 *  2. Missing entries before this one? Reply no with our last index so the
 *     leader goes back and sends those first.
 *  3. Entry before this one has a different term? Our log disagrees with
 *     the leader's there, reply no so the leader backs up one more.
 *  4. Otherwise store the entry (throwing away anything after it that
 *     doesn't match the leader), update our commit index, reply yes.
 */
static void handle_append_entries(struct raft_node *n, uint8_t s, const uint8_t d[8], uint64_t t)
{
	struct raft_append a;
	uint16_t prev, match;
	char str[24];

	raft_unpack_append(d, &a);
	if (!heard_from_leader(n, s, a.term, t)) {
		if (a.term < n->term) {
			send_append_reply(n, false, n->last_index); /* tells the old leader our term */
		}
		return;
	}

	/* index of the entry before this one (empty APPEND: the leader's last entry) */
	prev = a.entry == RAFT_E_NONE ? a.index : (uint16_t)(a.index - 1);
	if (a.entry != RAFT_E_NONE && a.index == 0) {
		return; /* bad frame */
	}
	if (prev > n->last_index) {
		send_append_reply(n, false, n->last_index); /* we're missing entries, tell leader where we are */
		return;
	}
	if (n->log[prev].term != a.prev_term) {
		send_append_reply(n, false, prev ? (uint16_t)(prev - 1) : 0); /* logs don't match, go back one */
		return;
	}

	match = prev;
	if (a.entry != RAFT_E_NONE) {
		/* we have something different at this index (from an old leader): delete it and everything after */
		if (a.index <= n->last_index && n->log[a.index].term != a.entry_term) {
			debug_print(n, "conflict at #%u, deleting %u entries", a.index,
				    n->last_index - a.index + 1);
			n->last_index = (uint16_t)(a.index - 1);
		}
		if (a.index == n->last_index + 1) {
			if (a.index > RAFT_LOG_SIZE) {
				debug_print(n, "log full");
				return;
			}
			n->log[a.index].term = a.entry_term;
			n->log[a.index].entry = a.entry;
			n->last_index = a.index;
			debug_print(n, "append #%u %s", a.index, raft_entry_str(a.entry, str, sizeof(str)));
			if (a.commit_back == 0) {
				n->replay_until = a.index; /* leader already committed this, so it's old news */
			}
		}
		match = a.index;
	}

	/* commit up to the leader's commit index, but not past what we actually have */
	if (a.commit_back != RAFT_COMMIT_UNKNOWN) {
		uint16_t leader_commit = (uint16_t)(a.index - a.commit_back);
		uint16_t c = leader_commit < match ? leader_commit : match;

		if (c > n->commit_index) {
			n->commit_index = c;
		}
	}
	send_append_reply(n, true, match);
}

/*
 * Leader: APPEND_ACK from follower s. Records how much of the log s has
 * (match_index) and what to send it next (next_index), then checks if
 * anything new can be committed. On a "no", backs next_index up to where s
 * said it is.
 */
static void handle_append_reply(struct raft_node *n, uint8_t s, const uint8_t d[8], uint64_t t)
{
	struct raft_ack a;

	raft_unpack_ack(d, &a);
	if (a.term > n->term) {
		become_follower(n, a.term, 0, t); /* someone has a newer term, we're not leader anymore */
		return;
	}
	if (n->role != RAFT_LEADER || a.term != n->term) {
		return;
	}
	n->last_ack_us[s] = t;
	if (a.success) {
		if (a.match > n->match_index[s]) {
			n->match_index[s] = a.match;
		}
		if (a.match + 1 > n->next_index[s]) {
			n->next_index[s] = (uint16_t)(a.match + 1);
		}
		update_commit_index(n, t);
	} else {
		uint16_t next = (uint16_t)(a.match + 1);

		if (next > n->last_index + 1) {
			next = (uint16_t)(n->last_index + 1);
		}
		n->next_index[s] = next < 1 ? 1 : next;
	}
}

/*
 * EVENT from node s: its client is reporting a change. The leader adds it
 * to the log. A follower that knows who the leader is sends REDIRECT back.
 */
static void handle_client_report(struct raft_node *n, uint8_t s, const uint8_t d[8], uint64_t t)
{
	uint8_t entry = raft_unpack_event(d);
	uint8_t out[8];

	(void)s;
	(void)t;
	if (n->role == RAFT_LEADER) {
		raft_leader_add_entry(n, entry);
	} else if (n->leader != 0) {
		struct raft_redirect r = {.term = n->term, .leader = n->leader};

		raft_pack_redirect(out, next_seq(n, TXS_REDIRECT), &r);
		send_frame(n, RAFT_ID_REDIRECT, out);
	}
}

/* ======================================================== public functions (see raft.h) */

/*
 * Sets everything up at boot. Three ways to start:
 *  - saved term from a file (Pi rebooted): use it, wait to hear the leader
 *  - default leader configured: everyone starts in term 1 with that leader
 *  - no default leader: start in term 0 and hold an election once the timer runs out
 */
void raft_init(struct raft_node *n, uint8_t my_id, const struct raft_port *port,
	       const struct raft_opts *opts, const struct raft_saved *saved)
{
	uint64_t t;

	memset(n, 0, sizeof(*n));
	n->id = my_id;
	n->port = port;
	n->opts.default_leader = opts ? opts->default_leader : RAFT_DEFAULT_LEADER;
	n->opts.seed = opts ? opts->seed : RAFT_SEED;
	t = get_time_us(n);

	/* mix the node ID into the seed so the 3 nodes get different random numbers */
	n->rng = n->opts.seed * 0x9E3779B1u ^ (uint32_t)my_id * 0x85EBCA6Bu;
	if (n->opts.default_leader == RAFT_LEADER_NONE) {
		n->rng ^= (uint32_t)t ^ (uint32_t)(t >> 32);
	}
	if (n->rng == 0) {
		n->rng = 1; /* xorshift gets stuck at 0 */
	}

	sm_init(&n->net);
	n->pending_net = n->net;
	n->role = RAFT_FOLLOWER;

	if (saved != NULL && saved->term != 0) {
		n->term = saved->term;
		n->voted_for = saved->voted_for;
		debug_print(n, "restored term %u", n->term);
	} else if (is_valid_node(n->opts.default_leader)) {
		n->term = 1;
		n->voted_for = n->opts.default_leader;
		n->leader = n->opts.default_leader;
		save_term_and_vote(n);
		if (my_id == n->opts.default_leader) {
			become_leader(n, t);
		}
	}
	n->deadline_us = t + election_timeout(n);
}

/*
 * Call every 1-5 ms.
 *  - leader: heartbeat every 20 ms, send log entries, check we still have a majority
 *  - follower: if the leader's been quiet too long, start a pre-vote
 *  - everyone: apply newly committed entries to the network graph
 */
void raft_tick(struct raft_node *n)
{
	uint64_t t = get_time_us(n);

	if (n->role == RAFT_LEADER) {
		if (t >= n->next_ref_us) {
			send_heartbeat(n);
			n->next_ref_us += MS(RAFT_HEARTBEAT_MS);
			if (n->next_ref_us <= t) {
				n->next_ref_us = t + MS(RAFT_HEARTBEAT_MS); /* fell behind, reset */
			}
		}
		send_log_to_followers(n, t);
		check_majority_alive(n, t);
	} else if (elections_allowed(n) && t >= n->deadline_us) {
		start_prevote(n, t);
	}
	apply_committed_entries(n);
}

/*
 * Call for every CAN frame received, on either bus. Ignores non-raft frames
 * and our own. Drops duplicate copies, then calls the handler for that
 * message type.
 */
void raft_receive_frame(struct raft_node *n, uint16_t id, const uint8_t d[8])
{
	uint8_t s;
	enum raft_msg m = raft_msg_from_id(id, &s);
	uint64_t t;

	if (m == RAFT_MSG_NONE || s == n->id) {
		return;
	}
	t = get_time_us(n);
	/* every frame shows up once per bus (and again if relayed) with the same
	 * seq, so only handle the first one. same idea as lab 3 */
	if (!can_seq_accept(&n->rx_filter[m][s], raft_seq(d), (uint32_t)(t / 1000))) {
		return;
	}
	switch (m) {
	case RAFT_MSG_REF:        handle_heartbeat(n, s, d, t); break;
	case RAFT_MSG_REQ_VOTE:   handle_vote_request(n, s, d, t); break;
	case RAFT_MSG_VOTE:       handle_vote_reply(n, s, d, t); break;
	case RAFT_MSG_APPEND:     handle_append_entries(n, s, d, t); break;
	case RAFT_MSG_APPEND_ACK: handle_append_reply(n, s, d, t); break;
	case RAFT_MSG_EVENT:      handle_client_report(n, s, d, t); break;
	default:                  break; /* REDIRECT goes to the client, not raft */
	}
	apply_committed_entries(n);
}

/*
 * Leader only. Adds a client's report to the log. It goes out to the
 * followers on the next tick and gets applied once a majority has it.
 * Skips reports that wouldn't change anything, since N2 and N3 will both
 * report the same link going down and it should only be logged once.
 */
int raft_leader_add_entry(struct raft_node *n, uint8_t entry)
{
	struct net_state probe;
	enum raft_ekind k = raft_entry_decode(entry).kind;
	uint16_t idx;
	char s[24];

	if (!raft_entry_submittable(entry)) {
		return RAFT_ERR_INVALID;
	}
	if (n->role != RAFT_LEADER) {
		return RAFT_ERR_NOT_LEADER;
	}
	probe = n->pending_net;
	if (k != RAFT_EK_EVENT && !sm_apply(&probe, entry)) {
		return 0; /* already in the log */
	}
	idx = log_append(n, entry);
	if (idx == 0) {
		return RAFT_ERR_LOG_FULL;
	}
	debug_print(n, "append #%u %s", idx, raft_entry_str(entry, s, sizeof(s)));
	n->next_append_us = 0; /* send it on the next tick */
	return idx;
}

/*
 * For the client. If we're leader, add it to the log directly. Otherwise
 * send an EVENT frame and the leader adds it.
 */
void raft_report_change(struct raft_node *n, uint8_t entry)
{
	uint8_t d[8];

	if (n->role == RAFT_LEADER) {
		raft_leader_add_entry(n, entry);
		return;
	}
	raft_pack_event(d, next_seq(n, TXS_EVENT), entry);
	send_frame(n, RAFT_ID_EVENT, d);
}

/* Copies out the committed network graph. */
void getNetwork(const struct raft_node *n, struct net_state *out) { *out = n->net; }

bool raft_log_get(const struct raft_node *n, uint16_t index, struct raft_log_entry *out)
{
	if (index < 1 || index > n->last_index) {
		return false;
	}
	*out = n->log[index];
	return true;
}
