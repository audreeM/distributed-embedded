#include <stdint.h>

typedef enum { 
	ST_ERROR = 0, 
	ST_NORMAL = 1, 
	ST_LOCAL_FAIL = 3,
} zone_state_t;

#define R_NONE 0x00
#define R_SELF_TEST_FAILED 0x01
#define R_HEARTBEAT_LOST 0x02
#define R_OUT_OF_RANGE_CTL_VAL 0x03
#define R_MALFORMED_MSG 0x04
#define R_ISOLATED 0x05
#define R_WATCHDOG_RESET 0x06
#define R_UDP_STREAM_LOST 0x07 // cockpit lost UDP
#define R_MOTOR_OPEN_CIRCUIT 0x08
#define R_MOTOR_OVERCURRENT 0x09
#define R_STUCK_WHEEL 0x0A
#define R_POWER_UP 0x0B // on start up the system is error state

void state_init(void);
void enter_error(uint8_t reasonUpdated);
void enter_normal(void);