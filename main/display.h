#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "stdbool.h"
#include "can_network.h"
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




extern can_msg_t read_display;
extern can_msg_t write_display;

void dwin_init_pages(void);
void write_error_msg(void);
uint16_t dwin_can_read(uint16_t addr);
void dwin_can_write(uint16_t addr, uint16_t value);
void display_error_msg(bool addr_flag,const char *error_data);
void switch_to_page(uint8_t page_number);
void clear_screen(uint16_t addr);

void dwin_can_rx_handler(const can_msg_t *msg);

#endif
