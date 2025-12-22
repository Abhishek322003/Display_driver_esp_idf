#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "stdbool.h"
#include "can_network.h"
#include "esp_event.h"
extern uint16_t error_flags,display_feedback;
extern volatile bool ccs2_gun_status_flag,ccs2_write_meter_flag;
//extern bool unplugged_sent;

#define UNPLUGGED 1
#define PLUGGED 2
#define START_CHARGING 3
#define STOP_CHARGING 4
#define ERROR_SCREEN 5
#define CCS2_SP_ADDR 0X9000
#define GUN2_SP_ADDR 0X8000
#define AC_SP_ADDR 0X7000





#define CCS2_GUN1_ADDR 0X1000
#define CCS2_GUN1_S_STOP_ADDR  0x1100
#define CCS2_GUN1_ERROR_ADDR 0X1050
#define TYPE6_GUN2_ADDR 0X2000
#define TYPE6_GUN2_S_STOP_ADDR 0X2100
///CCS2 GUN1 ADDRESES 
#define CCS2_ADDR_LOWER  0X1500
#define CCS2_ADDR_UPPER 0X1555
#define DISPLAY_TERMINATOR 0XFFFF



#define page_switch_addr        0x0085
#define page_switch_initaddr    0x0084


#define DWIN_READ_ID            0x0100
#define DWIN_WRITE_ID           0x0100

#define READ_CMD                0x83
#define write_cmd               0x82

#define line_break_addr 0x5020
#define line_break_data 0x0D0A

ESP_EVENT_DECLARE_BASE(DISPLAY_EVENT);

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
    uint16_t value;
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
void send_display_event(void* event);
void send_display_value_event(display_events_t event, uint16_t value);
void dwin_init_pages(void);
void write_error_msg(void);
uint16_t dwin_can_read(uint16_t addr);
void dwin_can_write(uint16_t addr, uint16_t value);
void display_error_msg(bool addr_flag,const char *error_data);
void switch_to_page(uint8_t page_number);
void clear_screen(uint16_t addr);
void dwin_can_hide_write(uint16_t addr,uint16_t random_data);
void display_event_handler(void *arg,esp_event_base_t base,int32_t event_id,void* event_data);
void dwin_can_rx_handler(const can_msg_t *msg);
void hide_data(uint16_t sp_addr);
void remove_hide(uint16_t sp_addr,uint16_t vp_addr);
void display_init();

#endif
