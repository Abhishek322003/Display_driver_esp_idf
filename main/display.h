#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "stdbool.h"
#include "can_network.h"
#include "esp_event.h"
extern uint16_t error_flags;
extern uint8_t start_charging, ready_for_charging;

#define page_switch_addr        0x0085
#define page_switch_initaddr    0x0084

#define start_stop_addr  0x1000

#define dwin_read_id            0x0100
#define dwin_write_id           0x0100

#define read_cmd                0x83
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


extern can_msg_t read_display;
extern can_msg_t write_display;
void send_display_event(display_events_t event);
void send_display_value_event(display_events_t event, uint16_t value);
void dwin_init_pages(void);
void write_error_msg(void);
uint16_t dwin_can_read(uint16_t addr);
void dwin_can_write(uint16_t addr, uint16_t value);
void display_error_msg(bool addr_flag,const char *error_data);
void switch_to_page(uint8_t page_number);
void clear_screen(uint16_t addr);

void display_event_handler(void *arg,esp_event_base_t base,int32_t event_id,void* event_data);
void dwin_can_rx_handler(const can_msg_t *msg);

#endif
