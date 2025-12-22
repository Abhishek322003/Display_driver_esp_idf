#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "energy_meter.h"
#include "esp_event.h"
#include "error.h"
uint16_t error_flags;
const char *TAG="DISPLAY";
static uint16_t error_str_addr=ERR_STRING_ADDR;
static uint16_t error_num_addr=ERR_NUMBER_ADDR;

ESP_EVENT_DEFINE_BASE(DISPLAY_EVENT);

//esp_timer_handle_t timer_10min;
//esp_timer_handle_t timer_300ms; 
static esp_event_loop_handle_t display_event_loop = NULL;

can_msg_t read_display = {
    .id = DWIN_READ_ID,
    .dlc = 8,
    .buff = {0x06, READ_CMD, 0x00, 0x00, 0x00, 0x00}
};

can_msg_t write_display = {
    .id = DWIN_WRITE_ID,
    .dlc = 6,
    .buff = {0x05, write_cmd, 0x00, 0x00, 0x00, 0x00}
};
static void set_the_error_flag(uint16_t value){
    if (value > 0 && value < 12) {
		if (value == 1) {
			error_flags |= ERR_CHARGE_STOPPED;
		}
		else if (value == 2) {
			error_flags |= ERR_EMERGENCY_STOP;
		}
		else if (value == 3) {
			error_flags |= ERR_UNDER_VOLTAGE;
		}
		else if (value == 4) {
			error_flags |= ERR_COMMUNICATION;
		}
		else if (value == 5) {
			error_flags |= ERR_CHARGER_OVER_TEMPERATURE;
		}
		else if (value == 6) {
			error_flags |= ERR_EARTH_FAULT;
		}
		else if (value == 7) {
			error_flags |= ERR_CONTACTOR_FAULT;
		}
		else if (value == 8) {
			error_flags |= ERR_RELAY_WELD_FAULT;
		}
		else if (value == 9) {
			error_flags |= ERR_CP_INVALID;
		}
		else if (value == 10) {
			error_flags |= ERR_NETWORK_FAULT;
		}
		else if (value == 11) {
			error_flags |= ERR_OVER_VOLTAGE;
		}
	}
	else {
		ESP_LOGW(TAG, "Invalid error index: %d", value);
        return;
	}


}

void display_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
	(void)arg;
	(void)base;

	display_events_t event = (display_events_t)event_id;

	if (event > AC_SOCKET_DISCONNECTED && event_data == NULL) {
		ESP_LOGW(TAG, "Event %d received without data", event);
		return;
	}
	display_req_data_t *data = (display_req_data_t *)event_data;
	switch (event) {

	case CCS2_GUN_CONNECTED:
		ccs2_gun_status_flag = true;
		break;

	case CCS2_GUN_DISCONNECTED:
		dwin_can_write(CCS2_GUN1_ADDR, UNPLUGGED);
		ccs2_gun_status_flag = false;
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
		dwin_can_write(CCS2_INITIAL_SOC, data->value);
		break;

	case CURRENT_SOC:
		dwin_can_write(CCS2_CURRENT_SOC, data->value);
		break;

	case DEMAND_VOLTAGE:
		dwin_can_write(CCS2_DEMAND_VOLTAGE, data->value);
		break;

	case UNIT_RATE:
		dwin_can_write(CCS2_UNIT_RATE, data->value);
		break;

	case AMOUNT:
		dwin_can_write(CCS2_AMOUNT, data->value);
		break;

	case CHARGING_POWER:
		dwin_can_write(CCS2_CHARGING_POWER, data->value);
		break;

	case GUN_TEMPERATURE:
		dwin_can_write(CCS2_GUN_TEMPERATURE, data->value);
		break;
	case ERROR_DISPLAY_EVENT:
        ESP_LOGI("DISPLAY:","error event recieved with data %d",data->value);
        set_the_error_flag(data->value);
		break;
	default:
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


 void send_display_event(void* event)
{
    display_events_t ev=*(display_events_t *)event;
    esp_event_post_to(
        display_event_loop,
        DISPLAY_EVENT,
        ev,
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

static void reset_display(){
    dwin_can_write(CCS2_GUN1_ADDR,UNPLUGGED);
    dwin_can_write(0X2000,UNPLUGGED);
    dwin_can_write(0X3000,UNPLUGGED);
    dwin_can_write(0x5000,1);
    hide_data(CCS2_GUN1_ADDR);
    hide_data(GUN2_SP_ADDR);
    hide_data(AC_SP_ADDR);
}

void dwin_init_pages(void)
{
    can_network_register_rx_cb(TWAI_CAN, dwin_can_rx_handler);
    display_event_loop_init();
    switch_to_page(0);
    dwin_can_write(CCS2_GUN1_ADDR,UNPLUGGED);
    dwin_can_write(CCS2_GUN1_S_STOP_ADDR,0);
    reset_display();
    vTaskDelay(pdMS_TO_TICKS(2000));
    switch_to_page(8);//gun is in unplugged state
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

void dwin_can_hide_write(uint16_t addr, uint16_t value){
    write_display.buff[2] = (addr >> 8) & 0xFF;
    write_display.buff[3] = addr & 0xFF;
    write_display.buff[4] = (value >> 8) & 0xFF;
    write_display.buff[5] =value& 0xFF;

    can_network_transmit_std_id(TWAI_CAN, &write_display);
    vTaskDelay(pdMS_TO_TICKS(10));

}
void dwin_can_write(uint16_t addr, uint16_t value)
{
    write_display.buff[2] = (addr >> 8) & 0xFF;
    write_display.buff[3] = addr & 0xFF;
    write_display.buff[4] = (value >> 8) & 0xFF;
    write_display.buff[5] =value& 0xFF;

    can_network_transmit_std_id(TWAI_CAN, &write_display);
    ESP_LOGI(TAG, "Wrote value 0x%04X to address 0x%04X", value, addr);
    
}
//start stop command handler from display feedback
void dwin_can_rx_handler(const can_msg_t *msg)
{
    ESP_LOGI(TAG,"Received id:%04X %02X %02X %02X %02X %02X %02X %02X %02X",msg->id,msg->buff[0],msg->buff[1],msg->buff[2],msg->buff[3],msg->buff[4],msg->buff[5],msg->buff[6],msg->buff[7]);

    if(msg->buff[0]==0x06 && msg->buff[1]==READ_CMD){
        if(msg->buff[2]==((CCS2_GUN1_S_STOP_ADDR>>8)&0xFF) &&
           msg->buff[3]==(CCS2_GUN1_S_STOP_ADDR&0xFF)){
            
            if(msg->buff[6]){ // START
                ESP_LOGI(TAG,"start command received");
                display_feedback_flag.ccs2=true;
            }else{ // STOP
                ESP_LOGI(TAG,"stop command received");
                if(display_feedback_flag.ccs2_t){ // charging active
                    display_feedback_flag.ccs2=false;
                    ESP_LOGI(TAG,"gun is unplugged");
                }
            }
        }
    }
}

void hide_data(uint16_t addr){
    dwin_can_write(addr,0xff00);//0xff00 to hide the data 
}
void remove_hide(uint16_t addr,uint16_t vp_addr){
    dwin_can_write(addr,vp_addr);
}


void clear_screen(uint16_t addr){/*
    dwin_can_write(ERR_STRING_ADDR,DISPLAY_TERMINATOR);
    dwin_can_write(ERR_NUMBER_ADDR,DISPLAY_TERMINATOR);
    */
   for(uint16_t i=0;i<300;i++){
        dwin_can_write(addr+i,' ');
    }
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
    error_str_addr=ERR_STRING_ADDR;
    error_num_addr=ERR_NUMBER_ADDR;
}