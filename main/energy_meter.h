#ifndef ENERGY_METER_H
#define ENERGY_METER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"

extern esp_timer_handle_t timer_1000ms;
extern esp_timer_handle_t timer_300ms;
extern esp_timer_handle_t energy_meter_timer;


#define BAUDRATE 9600
typedef enum {
    state_A,state_B,state_C,state_D,state_E 
}cp_state;
    
#define DC_METER_SLAVE_ID        24
#define AC_METER_SLAVE_ID       10
#define MB_PORT_NUM     UART_NUM_2
#define MODBUS_TX_PIN          41
#define MODBUS_RX_PIN          40
#define MODBUS_RE_DE_PIN       4
#define MODBUS_BAUDRATE        9600

#define DC_REG_KWH         0
#define DC_REG_VOLT        4
#define DC_REG_CURR        6
#define DC_REG_WATT        8

typedef struct {
    float voltage1;
    float current1;
    float power1;
    float energy_kWh;

    float voltage2;
    float current2;
    float power2;

    float soc;
    float tempC;
} meter_data_t;

void energy_meter_init(void);
float readFloatID(uint8_t slave_id, uint16_t reg_start);
void push_energy_meter_data();

#endif
