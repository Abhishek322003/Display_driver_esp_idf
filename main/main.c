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

display_events_t main_event_1=GUN_CONNECTED;
esp_timer_handle_t timer_3sec;
static gun_type_t active_gun = CCS2;   

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
    if (strlen(cmd) < 3) {
        ESP_LOGW(TAG, "Invalid gun command: %s", cmd);
        return;
    }
    display_req_data_t data;
    display_events_t event;
    switch (cmd[1]) {
        case '1': data.gun_type = CCS2; break;
        case '2': data.gun_type = TYPE6; break;
        case '3': data.gun_type = AC_SOCKET; break;
        default:
            ESP_LOGW(TAG, "Invalid gun id: %s", cmd);
            return;
    }
    active_gun = data.gun_type;

    if (cmd[2] == '1') {
        event = GUN_CONNECTED;

        switch (data.gun_type) {
            case CCS2:      data.value = CCS2_GUN_CONNECTED;      break;
            case TYPE6:     data.value = TYPE6_CONNECTED;         break;
            case AC_SOCKET: data.value = AC_SOCKET_CONNECTED;     break;
        }
    }
    else if (cmd[2] == '0') {
        event = GUN_DISCONNECTED;

        switch (data.gun_type) {
            case CCS2:      data.value = CCS2_GUN_DISCONNECTED;   break;
            case TYPE6:     data.value = TYPE6_DISCONNECTED;      break;
            case AC_SOCKET: data.value = AC_SOCKET_DISCONNECTED;  break;
        }
    }
    else {
        ESP_LOGW(TAG, "Invalid gun state: %s", cmd);
        return;
    }
    send_display_event(event, data);

    ESP_LOGI(TAG,
             "Gun Event Sent: %s | gun=%d | mask=0x%04X",
             cmd,
             data.gun_type,
             data.value);
}

void handle_uart_error_input(char *cmd)
{
    char *token = strtok(cmd + 1, " ");

    while (token != NULL) {

        uint8_t err_no = atoi(token);

        if (err_no >= 1 && err_no <= 11) {

            display_req_data_t data = {
                .gun_type = active_gun,
                .value    = err_no
            };

            send_display_event(ERROR_DISPLAY_EVENT, data);

            ESP_LOGI(TAG,
                     "Error Event Sent: e%d | gun=%d",
                     err_no,
                     active_gun);
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
        int len = uart_read_bytes(
            UART_NUM,
            buf,
            sizeof(buf) - 1,
            pdMS_TO_TICKS(100)
        );
        if (len <= 0) {
            continue;
        }

        buf[len] = '\0';
        char *cmd = (char *)buf;
        cmd[strcspn(cmd, "\r\n")] = 0;

        if (strlen(cmd) == 0) {
            continue;
        }

        ESP_LOGI(TAG, "CMD: %s", cmd);
        char *token = strtok(cmd, " ");
        while (token != NULL) {
            if (token[0] == 'g' || token[0] == 'G') {
                handle_uart_gun_input(token);
            }
            else if (token[0] == 'e' || token[0] == 'E') {
                handle_uart_error_input(token);
            }
            else {
                ESP_LOGW(TAG, "Unknown token: %s", token);
            }

            token = strtok(NULL, " ");
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

    

