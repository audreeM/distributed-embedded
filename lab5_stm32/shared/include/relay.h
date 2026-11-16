#ifndef RELAY_H
#define RELAY_H

#include "canbus.h"

/* Copy a frame onto to_bus (relay thread). Raft frames should already have RAFT_FLAG_RELAYED set. */
void relay_queue(const struct can_frame *f, int to_bus);
uint32_t relay_count(void);
int relay_init(void);

#endif /* RELAY_H */
