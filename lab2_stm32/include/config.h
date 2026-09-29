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

// thread IDs, each defined (dormant) in the file that owns the thread
// and started from main() once the safe state is set up
extern const k_tid_t cmd_tid;
extern const k_tid_t motor_tid;
extern const k_tid_t status_tid;
extern const k_tid_t sense_tid;

#define CONTROL_PERIOD_MS   10       

#define COUNTS_PER_REV      1317.1f  
#define ENC_L_INVERT        0        
#define ENC_R_INVERT        1
#define MOTOR_L_INVERT      1        
#define MOTOR_R_INVERT      0

#define THROTTLE_MAX        1000     
#define VEL_MAX_RPM         70.0f    

#define KP                  4.0f     
#define KI                  20.0f
#define I_LIMIT             1000.0f  

#define DUTY_MAX            1000     

#endif
