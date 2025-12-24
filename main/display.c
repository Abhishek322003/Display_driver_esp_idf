#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "energy_meter.h"
#include "esp_event.h"
#include "driver/uart.h"
#include "energy_meter.h"
#include "esp_modbus_master.h"
uint16_t error_flags;
const char *TAG="DISPLAY";
display_feedback_t display_feedback_flag;
uint32_t timer_addr_data;
//esp_timer_handle_t timer_5sec;
volatile bool error_handler_flag=false;
static bool start_charging_sent=false;
static bool unplugged_sent=false;
volatile bool ccs2_gun_status_flag=false;
bool err_flag=false;
bool ccs2_error_screen_shown=false;
static uint16_t error_str_addr=ERR_STRING_ADDR;
static uint16_t error_num_addr=ERR_NUMBER_ADDR;

ESP_EVENT_DEFINE_BASE(DISPLAY_EVENT);
static esp_event_loop_handle_t display_event_loop = NULL;

can_msg_t read_display = {
    .id = DWIN_READ_ID,
    .dlc = 8,
    .buff = {0x06, READ_CMD, 0x00, 0x00, 0x00, 0x00}
};

can_msg_t write_display = {
    .id = DWIN_WRITE_ID,
    .dlc = 6,
    .buff = {0x05, WRITE_CMD, 0x00, 0x00, 0x00, 0x00}
};
void timer_3sec_error_handler(void *addr_data);

esp_timer_create_args_t ccs2_timer_args = {
        .callback = timer_3sec_error_handler,
        .arg = &timer_addr_data,
        .dispatch_method = ESP_TIMER_TASK,  
        .name = "ccs2_timer"
    };

void timer_3sec_error_handler(void *addr_data){
    uint32_t packed = *(uint32_t *)addr_data;
    uint16_t addr = (packed >> 16) & 0xFFFF;
    uint16_t data = packed & 0xFFFF;
    dwin_can_write(addr, data);
    ESP_LOGI(TAG,"TIMER ENDED ONLY ONCE %x",packed);
}

    
void create_timers(uint16_t addr,uint16_t data)
{
    timer_addr_data = ((uint32_t)(addr & 0xFFFF) << 16) | (data & 0xFFFF);
    ESP_LOGI(TAG,"timer addr:%4x %2x %2x",timer_addr_data,addr,data);
    esp_timer_create(&ccs2_timer_args, &timer_3sec);
}




static void hide_data(uint16_t addr){
    dwin_can_write(addr,0xff00);//0xff00 to hide the data 
}
static void remove_hide(uint16_t addr,uint16_t vp_addr){
    dwin_can_write(addr,vp_addr);
}


static void display_error_msg(bool error_addr_flag, const char *error_data)
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
     dwin_can_write(error_addr+inc,LINE_BREAK_DATA);
     inc++;
     if(error_addr_flag){
       error_str_addr=error_addr+inc;
       ESP_LOGI(TAG, "%x erroraddr",error_str_addr);
     }
    else{
       error_num_addr=error_addr+inc;
       ESP_LOGI(TAG, "%x erroraddr",error_num_addr);
    }
}


static void handle_ccs2_connected(void)
{
    unplugged_sent=false;
    if(!start_charging_sent){
        dwin_can_write(CCS2_GUN1_ADDR,PLUGGED);
        vTaskDelay(pdMS_TO_TICKS(2000));
        //create_timers(CCS2_GUN1_ADDR,START_CHARGING);
        //esp_timer_start_once(timer_3sec,2000000);
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
       // esp_timer_start_periodic(timer_5sec,5000000);
        display_feedback_flag.ccs2_t=true;
        remove_hide(CCS2_SP_ADDR,CCS2_GUN1_S_STOP_ADDR);
    }
    if(!display_feedback_flag.ccs2 && display_feedback_flag.ccs2_t){
        ESP_LOGI(TAG,"Charging stopped");
        //esp_timer_stop(timer_5sec);
        display_feedback_flag.ccs2_t=false;
        dwin_can_write(CCS2_GUN1_ADDR,START_CHARGING);
        hide_data(CCS2_SP_ADDR);
    }
}

static void handle_ccs2_unplugged(void)
{
    start_charging_sent=false;
    display_feedback_flag.ccs2=false;
    error_handler_flag=false;
    if(display_feedback_flag.ccs2_t){//handling the timer when it is unplugged
        //esp_timer_stop(timer_5sec);
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
           
        }else{
            handle_ccs2_unplugged();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

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

static void set_the_error_flag(uint16_t value){
    error_events_t number=(error_events_t)value;
    if (number<= OVER_VOLTAGE && number >=CHARGING_IS_STOPPED) {
		if (value ==CHARGING_IS_STOPPED ) {
			error_flags |= ERR_CHARGE_STOPPED;//1
		}
		else if (value == EMERGENCY_SWITCH) {
			error_flags |= ERR_EMERGENCY_STOP;//2
		}
		else if (value == UNDER_VOLTAGE) {
			error_flags |= ERR_UNDER_VOLTAGE;//4
		}
		else if (value == COMMUNICATION_ERROR) {
			error_flags |= ERR_COMMUNICATION;//8
		}
		else if (value == CHARGER_OVER_TEMPERATURE) {
			error_flags |= ERR_CHARGER_OVER_TEMPERATURE;//12
		}
		else if (value == EARTH_FAULT) {
			error_flags |= ERR_EARTH_FAULT;
		}
		else if (value == CONTACTOR_FAULT) {
			error_flags |= ERR_CONTACTOR_FAULT;
		}
		else if (value == RELAY_WELD_FAULT) {
			error_flags |= ERR_RELAY_WELD_FAULT;
		}
		else if (value == CP_ERROR) {
			error_flags |= ERR_CP_INVALID;
		}
		else if (value == NETWORK_ERROR) {
			error_flags |= ERR_NETWORK_FAULT;
		}
		else if (value == OVER_VOLTAGE) {
			error_flags |= ERR_OVER_VOLTAGE;
		}
        write_error_msg();
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
		dwin_can_write_float(CCS2_INITIAL_SOC, data->value);
        ESP_LOGI(TAG,"INTIAL SOC EVENT RECIEVED %f",data->value);
		break;

	case CURRENT_SOC:
		dwin_can_write_float(CCS2_CURRENT_SOC, data->value);
		break;

	case DEMAND_VOLTAGE:
		dwin_can_write_float(CCS2_DEMAND_VOLTAGE, data->value);
		break;

	case UNIT_RATE:
		dwin_can_write_float(CCS2_UNIT_RATE, data->value);
		break;

	case AMOUNT:
		dwin_can_write_float(CCS2_AMOUNT, data->value);
		break;

	case CHARGING_POWER:
		dwin_can_write_float(CCS2_CHARGING_POWER, data->value);
		break;

	case GUN_TEMPERATURE:
		dwin_can_write_float(CCS2_GUN_TEMPERATURE, data->value);
		break;
	case ERROR_DISPLAY_EVENT:
        ESP_LOGI("DISPLAY:","error event recieved with data %d",data->value);
        switch_to_page(4);
        dwin_can_write(CCS2_GUN1_ADDR,ERROR_SCREEN);
        set_the_error_flag(data->value);
		break;
	default:
		break;
	}
}

void send_display_event(void* event){
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

void send_display_value_event(display_events_t event, uint16_t value){
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
    dwin_can_write(PAGE_SWITCH_ADDR, page_number);
    dwin_can_write(PAGE_SWITCH_INITADDR, 0x5A01);
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

    ESP_ERROR_CHECK(esp_event_loop_create(&display_loop_args, &display_event_loop));

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
    can_network_register_rx_cb(TWAI_CAN, dwin_can_rx_handler);
    dwin_can_write(CCS2_GUN1_ADDR,UNPLUGGED);
    dwin_can_write(TYPE6_GUN2_ADDR,UNPLUGGED);
    dwin_can_write(AC_SOCKET_ADDR,UNPLUGGED);
    dwin_can_write(DISPLAY_HEADER,1);
    hide_data(CCS2_SP_ADDR);
    hide_data(GUN2_SP_ADDR);
    hide_data(AC_SP_ADDR);
}


void dwin_can_hide_write(uint16_t addr, uint16_t value){
    write_display.buff[2] = (addr >> 8) & 0xFF;
    write_display.buff[3] = addr & 0xFF;
    write_display.buff[4] = (value >> 8) & 0xFF;
    write_display.buff[5] =value& 0xFF;

    can_network_transmit_std_id(TWAI_CAN, &write_display);
    //vTaskDelay(pdMS_TO_TICKS(10));

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
        if(msg->buff[2]==((CCS2_GUN1_S_STOP_ADDR>>8)&0xFF) &&msg->buff[3]==(CCS2_GUN1_S_STOP_ADDR&0xFF)){
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
        else if(msg->buff[2]==((CCS2_GUN1_ADDR>>8)&0xFF)&&msg->buff[3]==(CCS2_GUN1_ADDR&0xFF)){
            if(msg->buff[6]==UNPLUGGED){
                hide_data(CCS2_SP_ADDR);
                switch_to_page(8);
            }

    }
}
}


static void clear_screen(uint16_t addr){
    dwin_can_write(ERR_STRING_ADDR,DISPLAY_TERMINATOR);
    dwin_can_write(ERR_NUMBER_ADDR,DISPLAY_TERMINATOR);
    /*
   for(uint16_t i=0;i<300;i++){
        dwin_can_write(addr+i,' ');
    }*/
}

void write_error_msg() {
    clear_screen(0x1800);
    clear_screen(0x1900);
    for(uint8_t i = 0; i < 12; i++) {
        if ((error_flags >> i) & 1) {
            dwin_can_write(CCS2_GUN1_ERROR_ADDR, 401 + i);//to write for once
            switch (i) {
                case 0:
                    display_error_msg(1, "CHARGING IS STOPPED");
                    display_error_msg(0, "401");
                    break;
                case 1:
                    display_error_msg(1, "EMERGENCY SWITCH PRESSED");
                    display_error_msg(0, "402");
                    break;
                case 2:
                    display_error_msg(1, "UNDER VOLTAGE");
                    display_error_msg(0, "403");
                    break;
                case 3:
                    display_error_msg(1, "COMMUNICATION ERROR");
                    display_error_msg(0, "404");
                    break;
                case 4:
                    display_error_msg(1, "CHARGER OVER TEMPERATURE");
                    display_error_msg(0, "405");
                    break;
                case 5:
                    display_error_msg(1, "EARTH FAULT");
                    display_error_msg(0, "406");
                    break;
                case 6:
                    display_error_msg(1, "CONTACTOR FAULT");
                    display_error_msg(0, "407");
                    break;
                case 7:
                    display_error_msg(1, "RELAY WELD FAULT");
                    display_error_msg(0, "408");
                    break;
                case 8:
                    display_error_msg(1, "CP ERROR");
                    display_error_msg(0, "409");
                    break;
                case 9:
                    display_error_msg(1, "NETWORK ERROR");
                    display_error_msg(0, "410");
                    break;
                case 10:
                    display_error_msg(1, "OVER VOLTAGE");
                    display_error_msg(0, "411");
                    break;
                default:
                    display_error_msg(1, "UNKNOWN ERROR");
                    display_error_msg(0, "412");
                    break;
            }
        }
    }
    dwin_can_write(error_str_addr, DISPLAY_TERMINATOR);
    dwin_can_write(error_num_addr, DISPLAY_TERMINATOR);
    error_str_addr = ERR_STRING_ADDR;
    error_num_addr = ERR_NUMBER_ADDR;
}

static void dwin_init_pages(void)
{
    reset_display();
    display_event_loop_init();
    switch_to_page(0);
    dwin_can_write(CCS2_GUN1_ADDR,UNPLUGGED);
    dwin_can_write(CCS2_GUN1_S_STOP_ADDR,0);
    //create_timers(CCS2_GUN1_ADDR,UNPLUGGED);
    //esp_timer_start_once(timer_3sec,2000000);
    vTaskDelay(pdMS_TO_TICKS(2000));
    switch_to_page(8);//gun is in unplugged state
   // switch_to_page(10);
}
void display_init(){
    uart_init();
    //create_timers();
    //energy_meter_init();
    dwin_init_pages();
    xTaskCreate(ccs2_task,"ccs2_gun1_task",4096,NULL,4,NULL);
    xTaskCreate(uart_task,"uart_task",4096,NULL,6,NULL);
    }

