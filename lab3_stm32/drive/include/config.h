#ifndef CONFIG_H
#define CONFIG_H

#define STACK_SZ 1024


#define PRIO_DISPATCH 1 
#define PRIO_MOTOR    2
#define PRIO_RELAY    3
#define PRIO_BLINK_R  4 
#define PRIO_TEST_BTN 5 
#define PRIO_PID      6
#define PRIO_HB_MON   7 
#define PRIO_HB_TX    8 


#define HB_PERIOD_MS      20  
#define HB_MISS_LIMIT     3   

#define DEGRADED_MS       40  
#define RELAY_AFTER_MS    60  
#define CTRL_STALE_MS     100 
#define GOOD_CTRL_TO_CLEAR 2 
#define FAULT_HOLD_MS     100 
#define FAULT_MIN_GAP_MS  20  
#define FORCE_PERIOD_MS   50  
#define HAZARD_HALF_MS    250 
#define SELFTEST_LOCKOUT_MS 50 
#define SELFTEST_DOUBLE_MS  500
#define WDT_TIMEOUT_MS    300 
#define WDT_FEED_MS       100 
#define STATUS_PRINT_MS   1000

#define MOTOR_L_INVERT      0        
#define MOTOR_R_INVERT      1
#define DUTY_MAX            1000     


#endif 