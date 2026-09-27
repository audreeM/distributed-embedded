#ifndef CONFIG_H_
#define CONFIG_H_


#define CONTROL_PERIOD_MS   10       /* Needs tweaking */

#define COUNTS_PER_REV      3960.0f  
#define ENC_L_INVERT        0        
#define ENC_R_INVERT        1
#define MOTOR_L_INVERT      0       
#define MOTOR_R_INVERT      1

#define THROTTLE_MAX        1000     /* throttle input range 0..1000 */
#define VEL_MAX_RPM         70.0f   

#define KP                  4.0f    
#define KI                  20.0f
#define I_LIMIT             1000.0f  

#define DUTY_MAX            1000   

#endif
