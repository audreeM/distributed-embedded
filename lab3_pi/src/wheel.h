/* Wheel stream from the proxy: one UDP packet -> CAN units. */
#ifndef WHEEL_H
#define WHEEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_catalog.h"

struct wheel {
	struct can_ctrl ctrl; /* steer/throttle/brake/buttons in CAN units */
	bool left, right;     /* turn-signal buttons as pressed right now */
	bool test;            /* cockpit self-test button as pressed right now */
};

/* Parse one UDP payload. Returns false if it is not a wheel packet. */
bool wheel_parse(const uint8_t *buf, size_t len, struct wheel *w);

#endif /* WHEEL_H */
