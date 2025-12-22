#ifndef ERROR_H
#define ERROR_H
#include <stdint.h>
#include "esp_event.h"
#include "esp_timer.h"
extern uint16_t error_flags ;
extern esp_timer_handle_t timer_3sec;

ESP_EVENT_DECLARE_BASE(ERROR_EVENT);
extern volatile bool err_flag;
typedef enum {
    CHARGING_IS_STOPPED=1,
    EMERGENCY_SWITCH,
    UNDER_VOLTAGE,
    COMMUNICATION_ERROR,
    CHARGER_OVER_TEMPERATURE,
    EARTH_FAULT,
    CONTACTOR_FAULT,
    RELAY_WELD_FAULT,
    CP_ERROR,
    NETWORK_ERROR,
    OVER_VOLTAGE,
}error_events_t ;

typedef struct {
    uint16_t value;
}error_req_data_t;

#define ERR_CHARGE_STOPPED (1<<0)
#define ERR_EMERGENCY_STOP (1<<1)
#define ERR_UNDER_VOLTAGE   (1<<2)
#define ERR_COMMUNICATION (1<<3)
#define ERR_CHARGER_OVER_TEMPERATURE (1<<4)
#define ERR_EARTH_FAULT  (1<<5)
#define ERR_CONTACTOR_FAULT (1<<6)
#define ERR_RELAY_WELD_FAULT (1<<7)
#define ERR_CP_INVALID (1<<8)
#define ERR_NETWORK_FAULT (1<<9)
#define ERR_OVER_VOLTAGE (1<<10)
void error_init();
#define UART_NUM         UART_NUM_0
#define UART_BUF_SIZE    128
void uart_init();
void send_error_event(void *);


#endif
