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

const char* tag="ERROR:";

ESP_EVENT_DEFINE_BASE(ERROR_EVENT);
static esp_event_loop_handle_t error_event_loop = NULL;

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
