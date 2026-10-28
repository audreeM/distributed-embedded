#ifndef RELAY_H
#define RELAY_H

#include "canbus.h"

void relay_queue(const struct can_frame *f, int to_bus);
uint32_t relay_count(void);
int relay_init(void);

#endif 