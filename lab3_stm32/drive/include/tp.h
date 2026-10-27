#ifndef TP_H
#define TP_H

#include <stdbool.h>

enum tp {
	TP_CAN_RX_A,
	TP_CAN_RX_B, 
	TP_CAN_TX_A, 
	TP_CAN_TX_B, 
	TP_DEG_A,    
	TP_DEG_B,    
	TP_RELAY_TX, 
	TP_ERR,      
	TP_COUNT,
};

int tp_init(void);
void tp_toggle(enum tp tp);
void tp_set(enum tp tp, bool on);

#endif 