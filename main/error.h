#ifndef ERROR_H
#define ERROR_H
#include <stdint.h>
extern uint16_t error_flags ;
#define ERR_STRING_ADDR 0X1000
#define ERR_NUMBER_ADDR 0X2000


#define emergency_switch_pin GPIO_NUM_15

#define ERR_CHARGE_STOPPED (1<<0)
#define ERR_EMERGENCY_STOP (1<<1)
#define ERR_CP_UNDER_VOLTAGE   (1<<2)
#define ERR_COMMUNICATION (1<<3)
#define ERR_CHARGER_OVER_TEMPERATURE (1<<4)
#define ERR_EARTH_FAULT  (1<<5)
#define ERR_CONTACTOR_FAULT (1<<6)
#define ERR_RELAY_WELD_FAULT (1<<7)
#define ERR_CP_INVALID (1<<8)
#define ERR_NETWORK_FAULT (1<<9)
#define ERR_OVER_VOLTAGE (1<<10)
void check_emergency_stop();
void error_init();

#endif
