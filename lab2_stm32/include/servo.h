#ifndef SERVO_H_
#define SERVO_H_

/* Calibration override from the shell: drive the servo at a raw pulse width
 * (SERVO_ABS_MIN_US..SERVO_ABS_MAX_US). us = 0 returns to following the wheel. */
void servo_set_raw_us(int us);

/* Pulse width currently being output, in microseconds. */
int servo_get_us(void);

#endif
