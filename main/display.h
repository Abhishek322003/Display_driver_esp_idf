#ifndef DISPLAY_H
#define DISPLAY_H
#include <stdint.h>
#include "stdbool.h"
#include "esp_event.h"
ESP_EVENT_DECLARE_BASE(DISPLAY_EVENT);
ESP_EVENT_DECLARE_BASE(TIMER_EVENT);
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
//TYPE6 ADDRESSES
#define TYPE6_INITIAL_SOC        0x2500
#define TYPE6_CURRENT_SOC        0x2505
#define TYPE6_DEMAND_VOLTAGE     0x250A
#define TYPE6_CHARGING_VOLTAGE   0x250F
#define TYPE6_DEMAND_CURRENT     0x2514
#define TYPE6_CHARGING_CURRENT   0x2519
#define TYPE6_DURATION           0x251E
#define TYPE6_UNIT_RATE          0x2523
#define TYPE6_AMOUNT             0x2528
#define TYPE6_CHARGING_POWER     0x252D
#define TYPE6_ENERGY_DELIVERED   0x2532
#define TYPE6_GUN_TEMPERATURE    0x2537

#define TYPE6_VOLTAGE_V1         0x2600
#define TYPE6_VOLTAGE_V2         0x2605
#define TYPE6_VOLTAGE_V3         0x260A
#define TYPE6_AVG_VOLTAGE        0x260F
#define TYPE6_CURRENT_I1         0x261E
#define TYPE6_CURRENT_I2         0x2623
#define TYPE6_CURRENT_I3         0x2628
#define TYPE6_AVG_CURRENT        0x262D
#define TYPE6_ACTIVE_POWER       0x2632
#define TYPE6_TOTAL_POWER        0x2637
#define TYPE6_FREQUENCY          0x2614
#define TYPE6_AVG_PF             0x2619
///AC_SOCKET ADDRESSES
#define AC_SOCKET_INITIAL_SOC        0x3500
#define AC_SOCKET_CURRENT_SOC        0x3505
#define AC_SOCKET_DEMAND_VOLTAGE     0x350A
#define AC_SOCKET_CHARGING_VOLTAGE   0x350F
#define AC_SOCKET_DEMAND_CURRENT     0x3514
#define AC_SOCKET_CHARGING_CURRENT   0x3519
#define AC_SOCKET_DURATION           0x351E
#define AC_SOCKET_UNIT_RATE          0x3523
#define AC_SOCKET_AMOUNT             0x3528
#define AC_SOCKET_CHARGING_POWER     0x352D
#define AC_SOCKET_ENERGY_DELIVERED   0x3532
#define AC_SOCKET_GUN_TEMPERATURE    0x3537

#define AC_SOCKET_VOLTAGE_V1         0x3600
#define AC_SOCKET_VOLTAGE_V2         0x3605
#define AC_SOCKET_VOLTAGE_V3         0x360A
#define AC_SOCKET_AVG_VOLTAGE        0x360F
#define AC_SOCKET_CURRENT_I1         0x361E
#define AC_SOCKET_CURRENT_I2         0x3623
#define AC_SOCKET_CURRENT_I3         0x3628
#define AC_SOCKET_AVG_CURRENT        0x362D
#define AC_SOCKET_ACTIVE_POWER       0x3632
#define AC_SOCKET_TOTAL_POWER        0x3637
#define AC_SOCKET_FREQUENCY          0x3614
#define AC_SOCKET_AVG_PF             0x3619
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
    INITIAL_SOC,
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
