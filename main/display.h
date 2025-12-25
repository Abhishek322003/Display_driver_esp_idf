#ifndef DISPLAY_H
#define DISPLAY_H
#include <stdint.h>
#include "stdbool.h"
#include "esp_event.h"
ESP_EVENT_DECLARE_BASE(DISPLAY_EVENT);
ESP_EVENT_DECLARE_BASE(TIMER_EVENT);

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
    CCS2_GUN_CONNECTED,
    CCS2_GUN_DISCONNECTED,
    GUN2_TYPE6_CONNECTED,
    GUN2_TYPE6_DISCONNECTED,
    AC_SOCKET_CONNECTED,
    AC_SOCKET_DISCONNECTED,
    ERROR_DISPLAY_EVENT,
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
    PAGE1,
    PAGE2,
    PAGE8,
    DEFAULT
} display_timer_events_t;
typedef struct{
    bool ccs2;
    bool ccs2_t;
    bool type6;
    bool acsocket;
} display_feedback_t;
void dwin_can_write(uint16_t addr, uint16_t value);
void dwin_can_write_float(uint16_t vp_addr, float value);
void send_display_event(void* event);
void send_display_timer_event(void* event);
void send_display_value_event(display_events_t event, uint16_t value);
void display_init();
#endif
