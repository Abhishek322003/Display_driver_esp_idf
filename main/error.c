#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "error.h"  
#include "can_network.h"
#include "esp_mac.h"
#include "esp_log.h"

static uint16_t error_str_addr=ERR_STRING_ADDR;
static uint16_t error_num_addr=ERR_NUMBER_ADDR;
static const char *tag="ERROR:";
void check_emergency_stop()
{
    gpio_set_direction(emergency_switch_pin, GPIO_MODE_INPUT);
        if (!gpio_get_level(emergency_switch_pin)) {
        error_flags |= ERR_EMERGENCY_STOP;
    } else {
        error_flags &= ~ERR_EMERGENCY_STOP;
    }
}//1,2,3,4  ///communication error
void write_error_msg() {
    
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
    error_str_addr=ERR_STRING_ADDR;
    error_num_addr=ERR_NUMBER_ADDR;
}
//abcd
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

void gpio_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << emergency_switch_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);
}
void error_init(void)
{
    gpio_init();

}   
