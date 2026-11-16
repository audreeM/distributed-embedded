/*
 * The few things that differ between the two STM32s. The shared code calls
 * these; drive/src/node_drive.c and steering/src/node_steering.c implement them.
 */
#ifndef NODE_H
#define NODE_H

#include "can_catalog.h"

extern const char *const node_name;

/* Start the actuator threads in their safe state (motor braked / servo centered). */
int node_outputs_init(void);

/* A valid CTRL arrived (dispatch thread). */
void node_on_ctrl(const struct can_ctrl *c);

/* The zone state changed: re-check outputs now. */
void node_kick_outputs(void);

/* Extra periodic senders (steering: FORCE). */
int node_tx_init(void);

#endif /* NODE_H */
