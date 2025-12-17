#include <stdio.h>
#include "esp_system.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "can_network.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "display.h"
#include "esp_log.h"
#include "energy_meter.h"
#include "esp_modbus_master.h"
#include "error.h"

static const char *TAG = "MAIN";


uint8_t start_charging,ready_for_charging;
esp_timer_handle_t timer_1000ms;
esp_timer_handle_t timer_300ms; 

cp_state current_cp_state;
cp_state get_cp_status(){
    current_cp_state=state_C;
    return current_cp_state;
}
/*
void read_cp_voltage_from_vehicle(void *param)
{
    switch(get_cp_status()){
    case state_A:
        ESP_LOGI(TAG, "Vehicle disconnected, CP state: %d", state_A);
        ready_for_charging = 0;
        break;
    case state_B:
        ESP_LOGI(TAG, "Vehicle connected, CP state: %d", state_B);
        switch_to_page(2);//gun plugged
        vTaskDelay(pdMS_TO_TICKS(1000));
        switch_to_page(3);
        break;
    case state_C:
        ESP_LOGI(TAG, "Vehicle ready for charging, CP state: %d", state_C);
        ready_for_charging = 1;
        break;
    case state_D:
        ESP_LOGI(TAG, "Ventilation required %d", state_D);
        error_flags |= ERR_CP_UNDER_VOLTAGE;  
        ready_for_charging = 0;
        break; 
    case state_E:
        ESP_LOGI(TAG, "Short_circuit %d", state_E);
        ready_for_charging = 0;     
        break;

    }
}*/


void createTimers(void)
{
    /*const esp_timer_create_args_t cp_timer_args = {
        .callback = read_cp_voltage_from_vehicle,
        .dispatch_method = ESP_TIMER_ISR,  
        .name = "CAN1000ms"
    };*/
    const esp_timer_create_args_t emergency_timer_args = {
        .callback = check_emergency_stop,
        .dispatch_method = ESP_TIMER_ISR,  
        .name = "EMERGENCY300ms"
    };
   // esp_timer_create(&cp_timer_args, &timer_1000ms);//cp_voltage_timer
   // esp_timer_start_periodic(timer_1000ms, 1000000);
    
    esp_timer_create(&emergency_timer_args,&timer_300ms);//emergency_stop_timer
    esp_timer_start_periodic(timer_300ms, 300000);
   
}

void app_main(void)
{
    can_network_init();
    createTimers();
    error_init();
    //dwin_init_pages();
    while(1){
        send_error_event(CONTACTOR_FAULT);
        vTaskDelay(100);
        send_error_event(EARTH_FAULT);
        vTaskDelay(100);
        send_error_event(UNDER_VOLTAGE);
        vTaskDelay(100);
        send_error_event(CHARGER_OVER_TEMPERATURE);
        vTaskDelay(100);
        clear_screen(0x1600);
        clear_screen(0x1800);
        vTaskDelay(100);
        write_error_msg();
       
    }
}
