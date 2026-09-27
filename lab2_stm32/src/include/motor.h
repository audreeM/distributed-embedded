#ifndef MOTOR_H_
#define MOTOR_H_
#include <stdint.h>

int motor_init(void);
void motor_forward(uint16_t duty);   /* both motors, duty 0..DUTY_MAX */
/* TODO 3.2: motor_brake() - dynamic braking */

#endif