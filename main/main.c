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
error_events_t error_event=CHARGING_IS_STOPPED;
display_events_t main_event_1=CCS2_GUN_DISCONNECTED;
display_feedback_t display_feedback_flag;
esp_timer_handle_t timer_3sec;
volatile bool error_handler_flag=false;
static bool start_charging_sent=false,unplugged_sent=false;
volatile bool ccs2_gun_status_flag = false,ccs2_write_meter_flag=false,err_flag=false;
bool ccs2_error_screen_shown=false;


static void handle_ccs2_connected(void)
{
    unplugged_sent=false;
    if(!start_charging_sent){
        dwin_can_write(CCS2_GUN1_ADDR,PLUGGED);
        vTaskDelay(pdMS_TO_TICKS(2000));
        dwin_can_write(CCS2_GUN1_ADDR,START_CHARGING);
        ESP_LOGI(TAG,"START CHARGING screen shown");
        start_charging_sent=true;//executes only once
    }
}
static void handle_ccs2_start_charging(void)
{
    if(display_feedback_flag.ccs2 && !display_feedback_flag.ccs2_t){
        ESP_LOGI(TAG,"Charging started");
        dwin_can_write(CCS2_GUN1_ADDR,STOP_CHARGING);
        esp_timer_start_periodic(timer_3sec,3000000);
        display_feedback_flag.ccs2_t=true;
        remove_hide(CCS2_SP_ADDR,CCS2_GUN1_S_STOP_ADDR);
    }
    if(!display_feedback_flag.ccs2 && display_feedback_flag.ccs2_t){
        ESP_LOGI(TAG,"Charging stopped");
        esp_timer_stop(timer_3sec);
        display_feedback_flag.ccs2_t=false;
        dwin_can_write(CCS2_GUN1_ADDR,START_CHARGING);
        hide_data(CCS2_SP_ADDR);
    }
}

static void handle_ccs2_errors(void)
{
    if(!error_handler_flag) return;
    if(ccs2_error_screen_shown) return;
    switch_to_page(4);
    dwin_can_write(CCS2_GUN1_ADDR,5);
    uint8_t i;
    for(i=0;i<12;i++){
        if((error_flags>>i)&1) break;
    }
    dwin_can_write(CCS2_GUN1_ERROR_ADDR,400+i);
    write_error_msg();
    ccs2_error_screen_shown=true;
}

static void handle_ccs2_unplugged(void)
{
    start_charging_sent=false;
    display_feedback_flag.ccs2=false;
    error_handler_flag=false;
    if(display_feedback_flag.ccs2_t){//handling the timer when it is unplugged
        esp_timer_stop(timer_3sec);
        display_feedback_flag.ccs2_t=false;
        ESP_LOGI(TAG,"Error timer stopped");
    }
    if(!unplugged_sent){//writing only once 
        dwin_can_write(CCS2_GUN1_ADDR,UNPLUGGED);
        ESP_LOGI(TAG,"UNPLUGGED screen shown");
        unplugged_sent=true;
        hide_data(CCS2_SP_ADDR);
    }
}
void ccs2_task(void *arg)
{
    while(1){
        if(ccs2_gun_status_flag){//if gun connected 
            handle_ccs2_connected();
            handle_ccs2_start_charging();
            handle_ccs2_errors();
        }else{
            handle_ccs2_unplugged();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void timer_3sec_error_handler(){
    if(err_flag)
        error_handler_flag=true;
    else
        error_handler_flag=false;
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
        err_flag=true;
        uint8_t  err_no = atoi(token);
        if (err_no >= 1 && err_no <= 11) {
            error_event = (error_events_t)err_no;
            send_error_event(&error_event);
            ESP_LOGI(TAG, "Error Event Sent: e%d", err_no);
        } else {
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
            send_display_value_event(INTIAL_SOC, atoi(&cmd[2]));
        }
        else if (!strncmp(cmd, "cs", 2) || !strncmp(cmd, "CS", 2)) {
            send_display_value_event(CURRENT_SOC, atoi(&cmd[2]));
        }
        else if (!strncmp(cmd, "dv", 2) || !strncmp(cmd, "DV", 2)) {
            send_display_value_event(DEMAND_VOLTAGE, atoi(&cmd[2]));
        }
        else if (!strncmp(cmd, "ur", 2) || !strncmp(cmd, "UR", 2)) {
            send_display_value_event(UNIT_RATE, atoi(&cmd[2]));
        }
        else if (!strncmp(cmd, "am", 2) || !strncmp(cmd, "AM", 2)) {
            send_display_value_event(AMOUNT, atoi(&cmd[2]));
        }
        else if (!strncmp(cmd, "cp", 2) || !strncmp(cmd, "CP", 2)) {
            send_display_value_event(CHARGING_POWER, atoi(&cmd[2]));
        }
        else if (!strncmp(cmd, "gt", 2) || !strncmp(cmd, "GT", 2)) {
            send_display_value_event(GUN_TEMPERATURE, atoi(&cmd[2]));
        }
        else {
            ESP_LOGW(TAG, "Unknown command: %s", cmd);
        }
    }
}

void create_timers(void)
{
    const esp_timer_create_args_t ccs2_error_timer_args = {
        .callback = timer_3sec_error_handler,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,  
        .name = "ccs2_timer"
    };
    esp_timer_create(&ccs2_error_timer_args, &timer_3sec);
}

void display_init(){
    uart_init();
    create_timers();
    can_network_init();
    dwin_init_pages();
    //error_init();
    xTaskCreate(ccs2_task,"ccs2_gun1_task",4096,NULL,4,NULL);
    xTaskCreate(uart_task,"uart_task",4096,NULL,6,NULL);
    xTaskCreate(write_meter_task,"meter_task",4096,NULL,7,NULL);
    }

void app_main(void)
{
    display_init();
    ESP_LOGI("MAIN","Function is running");   
}

    

