/*
 * The raft thread: owns the raft_node (raft is single threaded), runs the
 * client, and publishes the routing table for the relay. Everything else
 * talks to it through these functions.
 */
#ifndef RAFT_TASK_H
#define RAFT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "raft.h"

/* A raft frame from dispatch (any copy, raft drops duplicates). */
void raft_task_post(uint16_t id, const uint8_t d[8]);

/* Ask the client to report this entry (e.g. EVENT for our own FAULT). */
void raft_task_report(uint8_t entry);

/* Relay decision from the routing table: the bus to copy the frame to, or -1. */
int raft_task_route(uint8_t sender, int from_bus);

/* Committed network says the cockpit is down (COCKPIT_ERROR). */
bool raft_task_failure_mode(void);

struct raft_status {
	enum raft_role role;
	uint8_t term, leader;
	uint16_t last_index, commit_index;
	struct net_state net;
};
void raft_task_status(struct raft_status *s);

/* Prints the log (console 'l') and a one-line status (console 's', main's status print). */
void raft_task_print_log(void);
void raft_task_print_status(void);

int raft_task_init(void);

#endif /* RAFT_TASK_H */
