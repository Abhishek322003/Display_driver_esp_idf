#ifndef DISPLAY_H
#define DISPLAY_H
#include <stdint.h>
#include "stdbool.h"
#include "can_network.h"
#include "esp_event.h"
#include "esp_timer.h"
ESP_EVENT_DECLARE_BASE(DISPLAY_EVENT);
extern uint32_t timer_addr_data;
extern uint16_t error_flags;
extern uint16_t display_feedback;
extern volatile bool ccs2_gun_status_flag;
extern uint16_t error_flags ;
extern esp_timer_handle_t timer_3sec;
#define ERR_STRING_ADDR 0X1900
#define ERR_NUMBER_ADDR 0X1800
#define UART_NUM         UART_NUM_0
#define UART_BUF_SIZE    128
//DISPLAY
#define PAGE_SWITCH_ADDR        0x0085
#define PAGE_SWITCH_INITADDR    0x0084
#define DWIN_READ_ID            0x0100
#define DWIN_WRITE_ID           0x0100
#define READ_CMD                0x83
#define WRITE_CMD               0x82
#define LINE_BREAK_ADDR 0x5020
#define LINE_BREAK_DATA 0x0D0A
#define DISPLAY_HEADER 0X5000
//COMMON
#define UNPLUGGED 1
#define PLUGGED 2
#define START_CHARGING 3
#define STOP_CHARGING 4
#define ERROR_SCREEN 5
//GUN1
#define CCS2_SP_ADDR 0X9000
#define CCS2_GUN1_ADDR 0X1000
#define CCS2_GUN1_S_STOP_ADDR  0x1100
#define CCS2_GUN1_ERROR_ADDR 0X1050
//GUN2
#define GUN2_SP_ADDR 0X8000
#define TYPE6_GUN2_ADDR 0X2000
#define TYPE6_GUN2_S_STOP_ADDR 0X2100

//GUN3
#define AC_SOCKET_ADDR 0X3000 
#define AC_SP_ADDR 0X7000


///CCS2 GUN1 ADDRESES 
#define CCS2_ADDR_LOWER  0X1500
#define CCS2_ADDR_UPPER 0X1555
#define DISPLAY_TERMINATOR 0XFFFF

//ERROR_FLAGS
//extern volatile bool err_flag;
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

typedef struct {
    float value;
} display_req_data_t;

typedef struct{
    bool ccs2;
    bool ccs2_t;
    bool type6;
    bool acsocket;
} display_feedback_t;
extern display_feedback_t display_feedback_flag;

extern can_msg_t read_display;
extern can_msg_t write_display;
void ccs2_task(void *arg);
void uart_task(void *arg);
void send_display_event(void* event);
void send_display_value_event(display_events_t event, uint16_t value);
void write_error_msg(void);
void dwin_can_write(uint16_t addr, uint16_t value);
void switch_to_page(uint8_t page_number);
void dwin_can_hide_write(uint16_t addr,uint16_t random_data);
void display_event_handler(void *arg,esp_event_base_t base,int32_t event_id,void* event_data);
void dwin_can_rx_handler(const can_msg_t *msg);
void display_init();
void create_timers(uint16_t addr,uint16_t data);
#endif
