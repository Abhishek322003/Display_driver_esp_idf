#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "error.h"  
#include "can_network.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_event.h"
#include "display.h"
#include "driver/uart.h"

static uint16_t error_str_addr=ERR_STRING_ADDR;
static uint16_t error_num_addr=ERR_NUMBER_ADDR;
const char* tag="ERROR:";

ESP_EVENT_DEFINE_BASE(ERROR_EVENT);
static esp_event_loop_handle_t error_event_loop = NULL;

void write_error_msg() {
    clear_screen(0x1800);
    clear_screen(0x1900);
    for(uint8_t i = 1; i < 12; i++) {
        if ((error_flags >> i) & 1) {
            switch (i) {
                case 1:
                    display_error_msg(1, "CHARGING IS STOPPED");display_error_msg(0, "401");
                    ESP_LOGI(tag, "401 CHARGING IS STOPPED");
                    break;
                case 2:
                    display_error_msg(1, "EMERGENCY SWITCH PRESSED");display_error_msg(0, "402");
                    ESP_LOGI(tag, "402 EMERGENCY SWITCH PRESSED");
                    break;
                case 3:
                    display_error_msg(1, "UNDER VOLTAGE");display_error_msg(0, "403");
                    ESP_LOGI(tag, "403 UNDER VOLTAGE");
                    break;
                case 4:
                    display_error_msg(1, "COMMUNICATION ERROR");display_error_msg(0, "404");
                    ESP_LOGI(tag, "404 COMMUNICATION ERROR");
                    break;
                case 5:
                    display_error_msg(1, "CHARGER OVER TEMPERATURE");display_error_msg(0, "405");
                    ESP_LOGI(tag, "405 CHARGER OVER TEMPERATURE");
                    break;
                case 6:
                    display_error_msg(1, "EARTH FAULT");display_error_msg(0, "406");
                    ESP_LOGI(tag, "406 EARTH FAULT");
                    break;
                case 7:
                    display_error_msg(1, "CONTACTOR FAULT");display_error_msg(0, "407");
                    ESP_LOGI(tag, "407 CONTACTOR FAULT");
                    break;
                case 8:
                    display_error_msg(1, "RELAY WELD FAULT");display_error_msg(0, "408");
                    ESP_LOGI(tag, "408 RELAY WELD FAULT");
                    break;
                case 9:
                    display_error_msg(1, "CP ERROR");display_error_msg(0, "409");
                    ESP_LOGI(tag, "409 CP ERROR");
                    break;
                case 10:
                    display_error_msg(1, "NETWORK ERROR");display_error_msg(0, "410");
                    ESP_LOGI(tag, "410 NETWORK ERROR");
                    break;
                case 11:
                    display_error_msg(1, "OVER VOLTAGE");display_error_msg(0, "411");
                    ESP_LOGI(tag, "411 OVER VOLTAGE");
                    break;
                default:
                    display_error_msg(1, "UNKNOWN ERROR");display_error_msg(0, "412");
                    ESP_LOGI(tag, "412 UNKNOWN ERROR");
                    break;
            }
        }
    }
    //clear_screen(ERR_STRING_ADDR);
    //clear_screen(ERR_NUMBER_ADDR);
    error_str_addr=ERR_STRING_ADDR;
    error_num_addr=ERR_NUMBER_ADDR;
}
//abcd
static void error_event_handler(void *arg,esp_event_base_t base,int32_t event_id,void *event_data){
    (void)arg;
    (void)base;
    error_events_t event = (error_events_t)event_id;
    switch (event) {
    case CHARGING_IS_STOPPED:
        error_flags|=ERR_CHARGE_STOPPED;
        break;
    case EMERGENCY_SWITCH:
        error_flags|=ERR_EMERGENCY_STOP;
        break;
    case UNDER_VOLTAGE:
        error_flags|=ERR_UNDER_VOLTAGE;
        break;
    case COMMUNICATION_ERROR:
        error_flags|=ERR_COMMUNICATION;
        break;
    case CHARGER_OVER_TEMPERATURE:
        error_flags|=ERR_CHARGER_OVER_TEMPERATURE;
        break;
    case EARTH_FAULT:
        error_flags|=ERR_EARTH_FAULT;
        break;
    case CONTACTOR_FAULT:
        error_flags|=ERR_CONTACTOR_FAULT;
        break;
    case RELAY_WELD_FAULT:
        error_flags|=ERR_RELAY_WELD_FAULT; 
        break;
    case CP_ERROR:
        error_flags|=ERR_CP_INVALID;
        break;
    case NETWORK_ERROR:
        error_flags|=ERR_NETWORK_FAULT;
        break;
    case OVER_VOLTAGE:
        error_flags|=ERR_OVER_VOLTAGE;
        break;
    default:
        break;
    }

}

static void error_event_loop_init(){
     esp_event_loop_args_t error_loop_args = {
        .queue_size = 10,
        .task_name = "error_task",
        .task_priority = 6,
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY
    };

    ESP_ERROR_CHECK(
        esp_event_loop_create(&error_loop_args, &error_event_loop)
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register_with(
            error_event_loop,
            ERROR_EVENT,
            ESP_EVENT_ANY_ID,
            error_event_handler,
            NULL
        )
    );
    ESP_LOGI(tag, "ERROR event loop created");
}

void send_error_event(void  *event)
{
    error_events_t ev = *(error_events_t *)event;
    esp_event_post_to(
        error_event_loop,
        ERROR_EVENT,
        ev,
        NULL,
        0,
        0
    );
    ESP_LOGI(tag,"ERROR_EVENT %d",event);
}

 void send_error_value_event(error_events_t event, uint16_t value)
{
    error_req_data_t data = {
        .value = value
    };

    esp_event_post_to(
        error_event_loop,
        ERROR_EVENT,
        event,
        &data,
        sizeof(data),
        0
    );
}

void display_error_msg(bool error_addr_flag, const char *error_data)
{
    uint16_t error_addr=0;
    uint8_t inc=0;
    if(error_addr_flag){
        error_addr=error_str_addr;
    }
    else{
        error_addr=error_num_addr;
    }
    while (*error_data && (*(error_data + 1)))
    {
        uint16_t send_data = *error_data<<8;//ABCD
        error_data++;
        send_data |= *error_data;
        dwin_can_write(error_addr + inc, send_data);//ABHISHEK
        error_data++;
        inc++;
    }
    if(*error_data){
        uint16_t space=*error_data<<8|0x20;
        dwin_can_write(error_addr+inc,space);
        inc++;
    }
     dwin_can_write(error_addr+inc,line_break_data);
     inc++;
     if(error_addr_flag){
       error_str_addr=error_addr+inc;
       ESP_LOGI(tag, "%x erroraddr",error_str_addr);
     }
    else{
       error_num_addr=error_addr+inc;
       ESP_LOGI(tag, "%x erroraddr",error_num_addr);
    }
}


void error_init(void)
{   
    dwin_can_write(0X1900,0xffff);//to clear the screen in one command
    dwin_can_write(0x1800,0xffff);
    ESP_LOGI("ERROR","SCREEN CLEARED");
    error_event_loop_init();
}   
void uart_init(){
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
