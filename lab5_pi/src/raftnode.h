/*
 * Raft on the Pi: the raft_node, its port (CAN + files), the client and the
 * routing table. Everything runs in the main poll() loop, so no locking.
 */
#ifndef RAFTNODE_H
#define RAFTNODE_H

#include <stdbool.h>
#include <stdint.h>

#include "canbus.h"
#include "raft.h"
#include "raft_client.h"

/*
 * warm = we heard a leader at startup, so this is the Pi coming back into a
 * running cluster: restore term/vote from RAFT_STATE_FILE and don't re-display
 * log entries we showed before (RAFT_LOG_FILE). cold = everyone is starting
 * up: start fresh, so LEADER=cockpit makes us leader in term 1.
 */
int raftnode_init(struct canbus *can, bool warm);
void raftnode_close(void);

/* The zone failed itself (test button): stop sending raft frames. */
void raftnode_set_silent(bool silent);

/* A raft frame from either bus (any copy). */
void raftnode_frame(uint16_t id, const uint8_t d[8]);

/* Every tick: raft_tick, and every CLIENT_MS the client + routing table. */
void raftnode_tick(const struct raft_obs *o, uint64_t now_us);

/* Relay decision: bus to copy a frame from `sender` onto, or -1. */
int raftnode_route(uint8_t sender, int from_bus);

/* Report an entry (our own FAULT as an EVENT, the 'e' key). */
void raftnode_report(uint8_t entry);

bool raftnode_is_leader(void);
void raftnode_print_status(void);
void raftnode_print_log(void);

#endif /* RAFTNODE_H */
