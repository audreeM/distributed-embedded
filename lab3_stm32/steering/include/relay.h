/* RELAY (Part 4.2): forwards a frame received on one bus onto the other. */
#ifndef RELAY_H
#define RELAY_H

#include "canbus.h"

/* Queue a frame for the RELAY thread to send on to_bus (dispatch thread). */
void relay_queue(const struct can_frame *f, int to_bus);
uint32_t relay_count(void);
int relay_init(void);

#endif /* RELAY_H */
