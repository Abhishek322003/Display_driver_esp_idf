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
void uart_task(void *arg);
#define UART_NUM         UART_NUM_0
#define UART_BUF_SIZE    128

static const char *TAG = "MAIN";

display_events_t main_event_1=CCS2_GUN_DISCONNECTED;
esp_timer_handle_t timer_3sec;

static void uart_init(){
    uart_config_t uart_conf = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM, UART_BUF_SIZE*2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_conf);
}

void handle_uart_gun_input(const char *cmd)
{
    display_events_t event;
    if (cmd[1] == '1') {                 
        event = (cmd[2] == '1') ? CCS2_GUN_CONNECTED: CCS2_GUN_DISCONNECTED;
    }
    else if (cmd[1] == '2') {            
        event = (cmd[2] == '1') ? GUN2_TYPE6_CONNECTED: GUN2_TYPE6_DISCONNECTED;
    }
    else if (cmd[1] == '3') {            
        event = (cmd[2] == '1') ? AC_SOCKET_CONNECTED: AC_SOCKET_DISCONNECTED;
    }
    else {
        ESP_LOGW(TAG, "Invalid gun command: %s", cmd);
        return;
    }
    main_event_1=event;
    send_display_event(&main_event_1);
    ESP_LOGI(TAG, "Gun Event Sent: %s", cmd);
}

void handle_uart_error_input(char *cmd)
{
    char *token = strtok(cmd + 1, " ");
    while (token != NULL) {
        uint8_t err_no = atoi(token);
        if (err_no >= 1 && err_no <= 11) {
            send_display_value_event(ERROR_DISPLAY_EVENT, err_no);
            ESP_LOGI(TAG, "Error Event Sent: e%d", err_no);
        } 
        else {
            ESP_LOGW(TAG, "Invalid error number: %s", token);
        }
        token = strtok(NULL, " ");
    }
}

void uart_task(void *arg)
{
    uint8_t buf[UART_BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_NUM,buf,sizeof(buf) - 1,pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }
        buf[len] = '\0';
        char *cmd = (char *)buf;
        cmd[strcspn(cmd, "\r\n")] = 0;
        ESP_LOGI(TAG, "CMD: %s", cmd);
        if (cmd[0] == 'g' || cmd[0] == 'G') {
            handle_uart_gun_input(cmd);
        }
        else if (cmd[0] == 'e' || cmd[0] == 'E') {
            handle_uart_error_input(cmd);
        }
        else if (!strncmp(cmd, "is", 2) || !strncmp(cmd, "IS", 2)) {
            send_display_value_event(INTIAL_SOC, atof(&cmd[2]));
        }
        else if (!strncmp(cmd, "cs", 2) || !strncmp(cmd, "CS", 2)) {
            send_display_value_event(CURRENT_SOC, atof(&cmd[2]));
        }
        else if (!strncmp(cmd, "dv", 2) || !strncmp(cmd, "DV", 2)) {
            send_display_value_event(DEMAND_VOLTAGE, atof(&cmd[2]));
        }
        else if (!strncmp(cmd, "ur", 2) || !strncmp(cmd, "UR", 2)) {
            send_display_value_event(UNIT_RATE, atof(&cmd[2]));
        }
        else if (!strncmp(cmd, "am", 2) || !strncmp(cmd, "AM", 2)) {
            send_display_value_event(AMOUNT, atof(&cmd[2]));
        }
        else if (!strncmp(cmd, "cp", 2) || !strncmp(cmd, "CP", 2)) {
            send_display_value_event(CHARGING_POWER, atof(&cmd[2]));
        }
        else if (!strncmp(cmd, "gt", 2) || !strncmp(cmd, "GT", 2)) {
            send_display_value_event(GUN_TEMPERATURE, atof(&cmd[2]));
        }
        else {
            ESP_LOGW(TAG, "Unknown command: %s", cmd);
        }
    }
}

void app_main(void)
{
    can_network_init();
    uart_init();
    //energy_meter_init();
    display_init();
    xTaskCreate(uart_task,"uart_task",4096,NULL,6,NULL);
    

}

    

