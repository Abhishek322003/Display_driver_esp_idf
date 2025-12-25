#ifndef DISPLAY_H
#define DISPLAY_H
#include <stdint.h>
#include "stdbool.h"
#include "esp_event.h"
ESP_EVENT_DECLARE_BASE(DISPLAY_EVENT);
ESP_EVENT_DECLARE_BASE(TIMER_EVENT);
#define CCS2_GUN_CONNECTED  (1<<0)
#define CCS2_GUN_DISCONNECTED (1<<1)
#define TYPE6_CONNECTED (1<<2) 
#define TYPE6_DISCONNECTED (1<<3)   
#define AC_SOCKET_CONNECTED  (1<<4)
#define AC_SOCKET_DISCONNECTED (1<<5) 

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

typedef enum{
    ERROR_DISPLAY_EVENT,
    GUN_CONNECTED,
    GUN_DISCONNECTED,
    INTIAL_SOC,
    CURRENT_SOC,
    DEMAND_VOLTAGE,
    UNIT_RATE,
    AMOUNT,
    CHARGING_POWER,
    GUN_TEMPERATURE
} display_events_t;

typedef enum{
    CCS2,
    TYPE6,
    AC_SOCKET
}gun_type_t;
typedef struct {
    uint16_t value;
    gun_type_t gun_type;
} display_req_data_t;

typedef enum{
    HOME_SCREEN,
    UNHIDE_CCS2_CHARGE_ADDR,
    PAGE1,
    PAGE2,
    PAGE8,
    DEFAULT
} display_timer_events_t;
void dwin_can_write(uint16_t addr, uint16_t value);
void dwin_can_write_float(uint16_t vp_addr, float value);
void send_display_event(display_events_t event,display_req_data_t data);
void send_display_timer_event(void* event);
void display_init();
#endif
