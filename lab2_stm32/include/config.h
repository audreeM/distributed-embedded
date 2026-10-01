#ifndef CONFIG_H
#define CONFIG_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#define STACK_SZ 1024

// TODO: check these values
#define PRIO_CMD 1
#define PRIO_MOTOR 2
#define PRIO_STATUS 3
#define PRIO_SENSE 4
#define PRIO_SERVO 3   /* 3.3: 50 ms budget */
#define PRIO_SIM 5     /* test shell: simulated Pi commands */
#define PRIO_BLINK 5   /* 3.4: 100 ms budget */

// thread IDs, each defined (dormant) in the file that owns the thread
// and started from main() once the safe state is set up
extern const k_tid_t cmd_tid;
extern const k_tid_t motor_tid;
extern const k_tid_t status_tid;
extern const k_tid_t sense_tid;
extern const k_tid_t servo_tid;
extern const k_tid_t blinker_tid;

#define CONTROL_PERIOD_MS   2       

#define COUNTS_PER_REV      1317.1f  
#define ENC_L_INVERT        0        
#define ENC_R_INVERT        1
#define MOTOR_L_INVERT      0        
#define MOTOR_R_INVERT      1

#define THROTTLE_MAX        1000     
#define VEL_MAX_RPM         70.0f    

#define KP                  4.0f     
#define KI                  20.0f
#define I_LIMIT             1000.0f  

#define DUTY_MAX            1000     

/* ---- 3.3 steering servo (LD-1501MG: 500..2500 us = 0..180 deg) ----
 * MEASURE with `servo us <n>`: find the pulse where the linkage just reaches
 * each end stop, then set MIN/MAX a little INSIDE those so it never buzzes.
 * The defaults below are deliberately conservative. */
#define SERVO_PERIOD_MS     20      /* 50 Hz frame, standard for hobby servos */
#define SERVO_MIN_US        800    /* MEASURE: full left  (steer = -100) */
#define SERVO_CENTER_US     1560    /* MEASURE: wheels straight (steer = 0) */
#define SERVO_MAX_US        2100    /* MEASURE: full right (steer = +100) */
#define SERVO_INVERT        1       /* 1 if wheel-left turns the car right */
#define SERVO_ABS_MIN_US    500     /* hard limits for raw calibration */
#define SERVO_ABS_MAX_US    2500

/* ---- 3.4 blinkers ----
 * Wheel range for self-cancel, in steer units (-100..+100). The wheel counts
 * as turned once |steer| >= ARM, and back in the middle once |steer| <= CANCEL.
 * The 2-count gap (hysteresis) stops it flickering right at the threshold.
 * From the Lab 1 design doc, 10.4. */
#define BLINK_PERIOD_MS     20      /* how often the blinker thread checks inputs */
#define BLINK_ARM_STEER     20
#define BLINK_CANCEL_STEER  18

#endif
