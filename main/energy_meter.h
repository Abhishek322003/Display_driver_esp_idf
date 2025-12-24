#ifndef ENERGY_METER_H
#define ENERGY_METER_H
#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"
#define BAUDRATE 9600
extern esp_timer_handle_t timer_10min;
extern esp_timer_handle_t timer_300ms;
extern esp_timer_handle_t energy_meter_timer;
typedef enum {
    state_A,state_B,state_C,state_D,state_E 
}cp_state;

#define CCS2_INITIAL_SOC        0x1500
#define CCS2_CURRENT_SOC        0x1505
#define CCS2_DEMAND_VOLTAGE     0x150A
#define CCS2_CHARGING_VOLTAGE   0x150F
#define CCS2_DEMAND_CURRENT     0x1514
#define CCS2_CHARGING_CURRENT   0x1519
#define CCS2_DURATION           0x151E
#define CCS2_UNIT_RATE          0x1523
#define CCS2_AMOUNT             0x1528
#define CCS2_CHARGING_POWER     0x152D
#define CCS2_ENERGY_DELIVERED   0x1532
#define CCS2_GUN_TEMPERATURE    0x1537

#define CCS2_VOLTAGE_V1         0x1600
#define CCS2_VOLTAGE_V2         0x1605
#define CCS2_VOLTAGE_V3         0x160A
#define CCS2_AVG_VOLTAGE        0x160F
#define CCS2_CURRENT_I1         0x161E
#define CCS2_CURRENT_I2         0x1623
#define CCS2_CURRENT_I3         0x1628
#define CCS2_AVG_CURRENT        0x162D
#define CCS2_ACTIVE_POWER       0x1632
#define CCS2_TOTAL_POWER        0x1637
#define CCS2_FREQUENCY          0x1614
#define CCS2_AVG_PF             0x1619


    
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
#define DC_REG_RST         16
#define METER_RST_VALUE     15
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
#define AC_ACTIVE_POWER 92
#define AC_REG_RST  178
/*
typedef struct
{
    uint16_t dc_kwh;     
    uint16_t dc_voltage;
    uint16_t dc_current;
    uint16_t dc_power;  

} dc_meter_data_t;
typedef struct
{
    uint16_t l1_voltage;       
    uint16_t l2_voltage;       
    uint16_t l3_voltage;       
    uint16_t avg_line_voltage; 
    uint16_t l1_current;       
    uint16_t l2_current;      
    uint16_t l3_current;      
    uint16_t avg_line_current;
    uint16_t l1_pf;            
    uint16_t l2_pf;            
    uint16_t l3_pf;            
    uint16_t system_pf;       
} ac_meter_data_t;
*/

void energy_meter_init(void);
//float read_float_id(uint8_t slave_id, uint16_t reg_start);
void push_energy_meter_data(void *arg);
//void write_random_data_to_display();
//void write_meter_task();
//void update_dc_meter();
void dwin_can_write_float(uint16_t vp_addr, float value);

#endif
