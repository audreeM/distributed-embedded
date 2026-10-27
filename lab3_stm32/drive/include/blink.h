/* BLINK_F: front blinkers FL / FR from the BLINK frames (R2.4). */
#ifndef BLINK_H
#define BLINK_H

#include "can_catalog.h"

/* A new, valid BLINK (dispatch thread). */
void blink_on_msg(const struct can_blink *b);
/* Re-evaluate now (state change). */
void blink_kick(void);
int blink_init(void);

#endif /* BLINK_H */