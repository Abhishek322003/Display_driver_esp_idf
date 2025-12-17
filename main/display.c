#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "energy_meter.h"
#include "esp_event.h"

uint16_t error_flags;
const char *TAG="DISPLAY";

ESP_EVENT_DEFINE_BASE(DISPLAY_EVENT);


static esp_event_loop_handle_t display_event_loop = NULL;





can_msg_t read_display = {
    .id = dwin_read_id,
    .dlc = 8,
    .buff = {0x06, read_cmd, 0x00, 0x00, 0x00, 0x00}
};

can_msg_t write_display = {
    .id = dwin_write_id,
    .dlc = 6,
    .buff = {0x05, write_cmd, 0x00, 0x00, 0x00, 0x00}
};

void display_event_handler(void *arg,esp_event_base_t base,int32_t event_id,void *event_data){
    (void)arg;
    (void)base;
    display_events_t event = (display_events_t)event_id;
    switch (event) {
    case CCS2_GUN_CONNECTED:
        dwin_can_write(0x2000, 0);
        break;
    case CCS2_GUN_DISCONNECTED:
        dwin_can_write(0x2000, 4);
        break;
    case GUN2_TYPE6_CONNECTED:
        dwin_can_write(0x2100, 0);
        break;
    case GUN2_TYPE6_DISCONNECTED:
        dwin_can_write(0x2100, 4);
        break;
    case AC_SOCKET_CONNECTED:
        dwin_can_write(0x2200, 0);
        break;
    case AC_SOCKET_DISCONNECTED:
        dwin_can_write(0x2200, 4);
        break;
    case INTIAL_SOC:
    case CURRENT_SOC:
    case DEMAND_VOLTAGE:
    case UNIT_RATE:
    case AMOUNT:
    case CHARGING_POWER:
    case GUN_TEMPERATURE:
    {
        if (event_data == NULL) {
            ESP_LOGW(TAG, "Event %d missing data", event);
            break;
        }

        display_req_data_t *data = (display_req_data_t *)event_data;

        switch (event) {
        case INTIAL_SOC:
            dwin_can_write(0x3000, data->value);
            break;

        case CURRENT_SOC:
            dwin_can_write(0x3002, data->value);
            break;

        case DEMAND_VOLTAGE:
            dwin_can_write(0x3004, data->value);
            break;

        case UNIT_RATE:
            dwin_can_write(0x3006, data->value);
            break;

        case AMOUNT:
            dwin_can_write(0x3008, data->value);
            break;

        case CHARGING_POWER:
            dwin_can_write(0x3010, data->value);
            break;

        case GUN_TEMPERATURE:
            dwin_can_write(0x3012, data->value);
            break;

        default:
            break;
        }
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown display event: %d", event);
        break;
    }
}
void write_cp_state_to_display(void* status){
    bool state=(bool *)status;
    if(state){
    switch_to_page(2);
    ESP_LOGI(TAG,"GUN CONNECTED WAIT FOR FEEDBACK");
    }
    else{
        switch_to_page(1);
        ESP_LOGI(TAG,"GUN DISCONNECTED CHARGING STOPPED");
    }
}


 void send_display_event(display_events_t event)
{
    esp_event_post_to(
        display_event_loop,
        DISPLAY_EVENT,
        event,
        NULL,
        0,
        0
    );
}

 void send_display_value_event(display_events_t event, uint16_t value)
{
    display_req_data_t data = {
        .value = value
    };

    esp_event_post_to(
        display_event_loop,
        DISPLAY_EVENT,
        event,
        &data,
        sizeof(data),
        0
    );
}

void switch_to_page(uint8_t page_number)
{
    dwin_can_write(page_switch_addr, page_number);
    dwin_can_write(page_switch_initaddr, 0x5A01);
    ESP_LOGI(TAG, "Switched to page %d", page_number);
}
static void display_event_loop_init(void)
{
    esp_event_loop_args_t display_loop_args = {
        .queue_size = 10,
        .task_name = "display_task",
        .task_priority = 5,
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY
    };

    ESP_ERROR_CHECK(
        esp_event_loop_create(&display_loop_args, &display_event_loop)
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register_with(
            display_event_loop,
            DISPLAY_EVENT,
            ESP_EVENT_ANY_ID,
            display_event_handler,
            NULL
        )
    );

    ESP_LOGI(TAG, "Display event loop created");
}



void dwin_init_pages(void)
{
    can_network_register_rx_cb(TWAI_CAN, dwin_can_rx_handler);
    display_event_loop_init();
    switch_to_page(0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    switch_to_page(1);
}

uint16_t dwin_can_read(uint16_t addr)
{
    read_display.buff[2] = (addr >> 8) & 0xFF;
    read_display.buff[3] = addr & 0xFF;

    can_network_transmit_std_id(TWAI_CAN, &read_display);

    vTaskDelay(pdMS_TO_TICKS(50));

    if (read_display.dlc == 8 &&
        read_display.buff[2] == ((addr >> 8) & 0xFF) &&
        read_display.buff[3] == (addr & 0xFF))
    {
        return (read_display.buff[4] << 8) | read_display.buff[5];
    }

    return 0;
}


void dwin_can_write(uint16_t addr, uint16_t value)
{
    write_display.buff[2] = (addr >> 8) & 0xFF;
    write_display.buff[3] = addr & 0xFF;
    write_display.buff[4] = (value >> 8) & 0xFF;
    write_display.buff[5] =value& 0xFF;

    can_network_transmit_std_id(TWAI_CAN, &write_display);
    ESP_LOGI(TAG, "Wrote value 0x%04X to address 0x%04X", value, addr);
    vTaskDelay(pdMS_TO_TICKS(10));
    
}
//start stop command handler from display feedback
void dwin_can_rx_handler(const can_msg_t *msg)
{
    ESP_LOGI(TAG, "Received CAN message with ID 0x%03X and DLC %d", msg->id, msg->dlc);
    if(msg->id==dwin_read_id && msg->dlc==8)
    {
        if(msg->buff[2]==start_stop_addr>>8&& msg->buff[3]==(start_stop_addr&0xFF))
        {
            uint16_t status=(msg->buff[4]<<8)|msg->buff[5];
            if(status)//chagnge the value of status from 0x10 to 0x1
            {
                ESP_LOGI(TAG, "Start command received\n");
                start_charging=1;
                switch_to_page(4); 
                esp_timer_start_periodic(energy_meter_timer,4000000);//write the data for every 4seconds
            }
            else
            {
                start_charging=0;
                switch_to_page(3);
                ESP_LOGI(TAG, "Stop command received\n");
                esp_timer_stop(energy_meter_timer);
            }
        }
    }
}


void clear_screen(uint16_t addr){
    for(uint16_t i=0;i<512;i++){
        dwin_can_write(addr+i,' ');
    }
}