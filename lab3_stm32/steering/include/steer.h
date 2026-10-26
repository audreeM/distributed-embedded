/* STEER: CTRL steering value -> servo PWM (R2.3). */
#ifndef STEER_H
#define STEER_H

#include "can_catalog.h"

void steer_on_ctrl(const struct can_ctrl *c);
void steer_kick(void);
int steer_init(void);

#endif /* STEER_H */
