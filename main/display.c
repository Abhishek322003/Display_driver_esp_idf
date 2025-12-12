#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "energy_meter.h"

uint16_t error_flags;
const char *TAG="DISPLAY";

esp_timer_handle_t energy_meter_timer;

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
void switch_to_page(uint8_t page_number)
{
    dwin_can_write(page_switch_addr, page_number);
    dwin_can_write(page_switch_initaddr, 0x5A01);
    ESP_LOGI(TAG, "Switched to page %d", page_number);
}


void dwin_init_pages(void)
{
    can_network_register_rx_cb(TWAI_CAN, dwin_can_rx_handler);
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