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

#endif
