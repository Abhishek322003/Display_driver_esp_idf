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
//dc_meter registers
#define DC_REG_KWH         0
#define DC_REG_VOLT        4
#define DC_REG_CURR        6
#define DC_REG_WATT        8
//ac_meter registers
#define AC_L1_VOL         20
#define AC_L2_VOL         24
#define AC_L3_VOL         28
#define AVG_LINE_VOL     32
#define AC_L1_CUR         52
#define AC_L2_CUR         56
#define AC_L3_CUR         60
#define AVG_LINE_CUR     64
#define AC_L1_PF 68
#define AC_L2_PF 70
#define AC_L3_PF 72
#define SYSTEM_PF 76
#define SYSTEM_FREQUENCY 78
#define active_power 92
typedef struct
{
    float dc_kwh;     
    float dc_voltage;
    float dc_current;
    float dc_power;  

} dc_meter_data_t;
typedef struct
{
    float L1_voltage;
    float L2_voltage;
    float L3_voltage;
    float avg_line_voltage;
    float L1_current;
    float L2_current;
    float L3_current;
    float avg_line_current;
    float L1_pf;
    float L2_pf;
    float L3_pf;
    float system_pf;

} ac_meter_data_t;

void energy_meter_init(void);
float readFloatID(uint8_t slave_id, uint16_t reg_start);
void push_energy_meter_data();

#endif
