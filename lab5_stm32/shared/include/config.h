/*
 * Settings for both STM32 nodes. NODE_ID comes from the app's CMakeLists.txt
 * (2 = steering, 3 = drivetrain). Raft settings are in common/raft/raft_config.h.
 */
#ifndef CONFIG_H
#define CONFIG_H

#ifndef NODE_ID
#error "NODE_ID must be set by CMakeLists.txt (2 = steering, 3 = drive)"
#endif

#define STACK_SZ      1024
#define STACK_SZ_RAFT 2048 /* raft debug prints use a 160-byte buffer */

/* Thread priorities, lower number = more urgent. */
#define PRIO_EDGES    0 /* PPS + blinker lamps: spins ~300 us before each edge */
#define PRIO_DISPATCH 1
#if NODE_ID == 3
#define PRIO_MOTOR    2
#define PRIO_RELAY    3
#define PRIO_PID      6
#else
#define PRIO_RELAY    2
#define PRIO_STEER    4
#define PRIO_FORCE_TX 8
#endif
#define PRIO_TEST_BTN 5
#define PRIO_HB_MON   7
#define PRIO_HB_TX    8
#define PRIO_RAFT     9
#define PRIO_CONSOLE  13
/* main() (status prints, watchdog) runs at CONFIG_MAIN_THREAD_PRIORITY = 12 so
 * its printk (several ms at 115200 baud) never holds up anything above */

/* Timing (ms), same as lab 3 */
#define HB_PERIOD_MS        20
#define HB_MISS_LIMIT       3
#define DEGRADED_MS         40
#define CTRL_STALE_MS       100
#define GOOD_CTRL_TO_CLEAR  2
#define FAULT_HOLD_MS       100
#define FAULT_MIN_GAP_MS    20
#define FORCE_PERIOD_MS     50
#define SELFTEST_LOCKOUT_MS 50
#define SELFTEST_DOUBLE_MS  500
#define WDT_TIMEOUT_MS      300
#define WDT_FEED_MS         100
#define STATUS_PRINT_MS     1000

/* Lab 5 */
#define HEALTH_FRESH_MS     60    /* a node's heartbeat health byte counts for this long */
#define RAFT_TICK_MS        2     /* raft_tick() period */
#define CLIENT_MS           10    /* rebuild obs / routing this often */
#define PEERS_READY_MAX_MS  30000 /* LEADER=random: start elections after this even if a node is missing */

/* Blinkers (half periods). Normal hazards are 2 Hz like lab 3; when the
 * cockpit is dead (failure mode) the STM32s run 4 Hz hazards on their own. */
#define BLINK_HALF_MS         500
#define HAZARD_HALF_MS        250
#define FAIL_HAZARD_HALF_MS   125

#define PPS_HIGH_MS 100 /* PPS pulse width */
#define EDGE_SPIN_US 300 /* wake up this early before an edge and busy-wait the rest */

#if NODE_ID == 3
#define MOTOR_L_INVERT 0
#define MOTOR_R_INVERT 1
#define DUTY_MAX       1000
#else
#define SERVO_MIN_US    800
#define SERVO_CENTER_US 1560
#define SERVO_MAX_US    2100
#define SERVO_INVERT    1
#endif

#endif /* CONFIG_H */
