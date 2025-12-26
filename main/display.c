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
#include "can_network.h"
//MACROS
#define ERR_STRING_ADDR 0X1900
#define ERR_NUMBER_ADDR 0X1800
//DISPLAY
#define PAGE_SWITCH_ADDR        0x0085
#define PAGE_SWITCH_INITADDR    0x0084
#define DWIN_READ_ID            0x0100
#define DWIN_WRITE_ID           0x0100
#define READ_CMD                0x83
#define WRITE_CMD               0x82
#define LINE_BREAK_ADDR 0x5020
#define LINE_BREAK_DATA 0x0D0A
#define DISPLAY_HEADER 0X5000
//COMMON
#define UNPLUGGED 1
#define PLUGGED 2
#define START_CHARGING 3
#define STOP_CHARGING 4
#define ERROR_SCREEN 5
//GUN1
#define CCS2_SP_ADDR 0X9000
#define CCS2_GUN1_ADDR 0X1000
#define CCS2_GUN1_S_STOP_ADDR  0x1100
#define CCS2_GUN1_ERROR_ADDR 0X1050
//GUN2
#define GUN2_SP_ADDR 0X8000
#define TYPE6_GUN2_ADDR 0X2000
#define TYPE6_GUN2_S_STOP_ADDR 0X2100
#define TYPE6_GUN2_ERROR_ADDR 0X2050
//GUN3
#define AC_SOCKET_ADDR 0X3000 
#define AC_SP_ADDR 0X7000
///CCS2 GUN1 ADDRESES 
///CCS2 GUN1 ADDRESES 
#define CCS2_ADDR_LOWER  0X1500
#define CCS2_ADDR_UPPER 0X1555
#define DISPLAY_TERMINATOR 0XFFFF
//ERROR_FLAGS
#define ERR_CHARGE_STOPPED (1<<0)
#define ERR_EMERGENCY_STOP (1<<1)
#define ERR_UNDER_VOLTAGE   (1<<2)
#define ERR_COMMUNICATION (1<<3)
#define ERR_CHARGER_OVER_TEMPERATURE (1<<4)
#define ERR_EARTH_FAULT  (1<<5)
#define ERR_CONTACTOR_FAULT (1<<6)
#define ERR_RELAY_WELD_FAULT (1<<7)
#define ERR_CP_INVALID (1<<8)
#define ERR_NETWORK_FAULT (1<<9)
#define ERR_OVER_VOLTAGE (1<<10)

ESP_EVENT_DEFINE_BASE(DISPLAY_EVENT);
ESP_EVENT_DEFINE_BASE(TIMER_EVENT);
static uint16_t error_flags;
static const char *TAG="DISPLAY";
static esp_timer_handle_t timer_3sec;
static display_timer_events_t current_page=DEFAULT;
static uint32_t timer_addr_data;
static uint16_t error_str_addr=ERR_STRING_ADDR;
static uint16_t error_num_addr=ERR_NUMBER_ADDR;
static esp_event_loop_handle_t display_event_loop = NULL;
static esp_event_loop_handle_t display_timer_event_loop=NULL;
/*static can_msg_t read_display = {
    .id = DWIN_READ_ID,
    .dlc = 8,
    .buff = {0x06, READ_CMD, 0x00, 0x00, 0x00, 0x00}
};*/
static can_msg_t write_display = {
    .id = DWIN_WRITE_ID,
    .dlc = 6,
    .buff = {0x05, WRITE_CMD, 0x00, 0x00, 0x00, 0x00}
};

static void set_the_error_flag(uint16_t value);
static void timer_3sec_handler(void *addr_data);
static void switch_to_page(uint8_t page_number);
static void write_error_msg();
static void display_timer_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);
static void display_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);

static esp_timer_create_args_t ccs2_timer_args = {
        .callback = timer_3sec_handler,
        .arg = &timer_addr_data,
        .dispatch_method = ESP_TIMER_TASK,  
        .name = "ccs2_timer"
    };

static inline void hide_data(uint16_t addr){
    dwin_can_write(addr,0xff00);//0xff00 to hide the data 
}

static inline void remove_hide(uint16_t addr,uint16_t vp_addr){
    dwin_can_write(addr,vp_addr);
}

static uint16_t get_gun_base_addr(gun_type_t gun){
    switch (gun) {
        case CCS2:      return CCS2_GUN1_ADDR;
        case TYPE6:     return TYPE6_GUN2_ADDR;
        case AC_SOCKET: return AC_SOCKET_ADDR;
        default:        return 0;
    }
}

static void display_timer_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
	(void)arg;
	(void)base;
	display_timer_events_t event = (display_timer_events_t)event_id;
	switch (event) {
	case DEFAULT:
        switch_to_page(8);
        ESP_LOGI(TAG,"PAGE8 EVENT");
		break;
    case UNHIDE_CCS2_CHARGE_ADDR:
        ESP_LOGI(TAG,"UNHIDE_CCS2_PAGE EVENT");
    case PAGE1: break;
    case PAGE2:break;
    case PAGE8:break;
    case HOME_SCREEN:
        ESP_LOGI(TAG,"PAGE1");
		break;

    }
}
static uint16_t get_display_addr(display_events_t event, gun_type_t gun){
    switch (event) {
    case INITIAL_SOC:
        if (gun == CCS2)       return CCS2_INITIAL_SOC;
        if (gun == TYPE6)      return TYPE6_INITIAL_SOC;
        if (gun == AC_SOCKET)  return AC_SOCKET_INITIAL_SOC;
        break;
    case CURRENT_SOC:
        if (gun == CCS2)       return CCS2_CURRENT_SOC;
        if (gun == TYPE6)      return TYPE6_CURRENT_SOC;
        if (gun == AC_SOCKET)  return AC_SOCKET_CURRENT_SOC;
        break;
    case DEMAND_VOLTAGE:
        if (gun == CCS2)       return CCS2_DEMAND_VOLTAGE;
        if (gun == TYPE6)      return TYPE6_DEMAND_VOLTAGE;
        if (gun == AC_SOCKET)  return AC_SOCKET_DEMAND_VOLTAGE;
        break;
    case CHARGING_POWER:
        if (gun == CCS2)       return CCS2_CHARGING_POWER;
        if (gun == TYPE6)      return TYPE6_CHARGING_POWER;
        if (gun == AC_SOCKET)  return AC_SOCKET_CHARGING_POWER;
        break;
    case UNIT_RATE:
        if (gun == CCS2)       return CCS2_UNIT_RATE;
        if (gun == TYPE6)      return TYPE6_UNIT_RATE;
        if (gun == AC_SOCKET)  return AC_SOCKET_UNIT_RATE;
        break;
    case AMOUNT:
        if (gun == CCS2)       return CCS2_AMOUNT;
        if (gun == TYPE6)      return TYPE6_AMOUNT;
        if (gun == AC_SOCKET)  return AC_SOCKET_AMOUNT;
        break;
    case GUN_TEMPERATURE:
        if (gun == CCS2)       return CCS2_GUN_TEMPERATURE;
        if (gun == TYPE6)      return TYPE6_GUN_TEMPERATURE;
        if (gun == AC_SOCKET)  return AC_SOCKET_GUN_TEMPERATURE;
        break;

    default:
        break;
    }
    return 0;  
}


static void write_display_event(display_events_t event,display_req_data_t *data,float f_data){
    uint16_t addr = get_display_addr(event, data->gun_type);
    if (addr == 0) {
        ESP_LOGW(TAG, "Invalid addr event=%d gun=%d", event, data->gun_type);
        return;
    }
    if (event == INITIAL_SOC || event == CURRENT_SOC) {
        dwin_can_write(addr, data->value);
    } else {
        dwin_can_write_float(addr, f_data);
    }
}

static void check_which_gun(display_req_data_t *data, bool connect)
{
    uint16_t addr = get_gun_base_addr(data->gun_type);

    ESP_LOGI(TAG, "Gun=%d value=%d connect=%d",
             data->gun_type, data->value, connect);

    if (addr == 0) {
        ESP_LOGW(TAG, "Invalid gun type");
        return;
    }

    dwin_can_write(addr, connect ? PLUGGED : UNPLUGGED);
    if (connect) {
        timer_addr_data = ((uint32_t)(addr & 0xFFFF) << 16) |
                          (START_CHARGING & 0xFFFF);
        ESP_LOGI(TAG, "Gun %d CONNECTED", data->gun_type);
    } else {
        ESP_LOGI(TAG, "Gun %d DISCONNECTED", data->gun_type);
    }
}


static void display_event_handler(void *arg,esp_event_base_t base,int32_t event_id,void *event_data)
{
    (void)arg;
    (void)base;
    display_events_t event = (display_events_t)event_id;
    display_req_data_t *data = (display_req_data_t *)event_data;
    float f_data = 0;
    if (event >= INITIAL_SOC && event <= GUN_TEMPERATURE) {
        uint16_t num  = data->value % 100;
        uint16_t num1 = data->value / 100;
        f_data = num + ((float)num1 / 100);
    }
    switch (event) {
    case GUN_CONNECTED:
        esp_timer_start_once(timer_3sec, 2000000);
        timer_addr_data = ((uint32_t)(CCS2_GUN1_ADDR & 0xFFFF) << 16) |(START_CHARGING & 0xFFFF);
        check_which_gun(data, true);
        break;
    case GUN_DISCONNECTED:
        check_which_gun(data, false);
        break;
    case INITIAL_SOC:
    case CURRENT_SOC:
    case DEMAND_VOLTAGE:
    case UNIT_RATE:
    case AMOUNT:
    case CHARGING_POWER:
    case GUN_TEMPERATURE:
        write_display_event(event, data, f_data);
        ESP_LOGI(TAG, "Display event %d value=%d f=%f",event, data->value, f_data);
        break;

    case ERROR_DISPLAY_EVENT:
        ESP_LOGI(TAG, "Error event %d", data->value);
        switch_to_page(4);
        dwin_can_write(CCS2_GUN1_ADDR, ERROR_SCREEN);
        set_the_error_flag(data->value);
        break;

    default:
        break;
    }
}


static void timer_3sec_handler(void *addr_data){
    uint32_t packed = *(uint32_t *)addr_data;
    uint16_t addr = (packed >> 16) & 0xFFFF;
    uint16_t data = packed & 0xFFFF;
    dwin_can_write(addr, data);
    ESP_LOGI(TAG,"TIMER ENDED ONLY ONCE %x",packed);
    esp_event_post_to(display_timer_event_loop,TIMER_EVENT,current_page,NULL,0, 0);
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

void send_display_timer_event(void* event){
   display_timer_events_t ev=*(display_timer_events_t *)event;
    esp_event_post_to(
        display_timer_event_loop,
        TIMER_EVENT,
        ev,
        NULL,
        0,
        0
    );
}
void send_display_event(display_events_t event,display_req_data_t data){

    esp_event_post_to(
        display_event_loop,
        DISPLAY_EVENT,
        event,
        &data,
        sizeof(display_req_data_t),
        portMAX_DELAY
    );
}


static void switch_to_page(uint8_t page_number)
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

static void display_timer_event_loop_init(void)
{
    esp_event_loop_args_t display_timer_loop_args = {
        .queue_size = 10,
        .task_name = "timer_task",
        .task_priority = 6,
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY
    };

    ESP_ERROR_CHECK(esp_event_loop_create(&display_timer_loop_args, &display_timer_event_loop));

    ESP_ERROR_CHECK(
        esp_event_handler_register_with(
            display_timer_event_loop,
            TIMER_EVENT,
            ESP_EVENT_ANY_ID,
            display_timer_event_handler,
            NULL
        )
    );
    ESP_LOGI(TAG, "display timer event loop created");
}

static void reset_display(){
    switch_to_page(0);//home screen
    dwin_can_write(CCS2_GUN1_ADDR,UNPLUGGED);
    dwin_can_write(CCS2_GUN1_S_STOP_ADDR,0);
    dwin_can_write(TYPE6_GUN2_ADDR,UNPLUGGED);
    dwin_can_write(AC_SOCKET_ADDR,UNPLUGGED);
    dwin_can_write(DISPLAY_HEADER,1);
    hide_data(CCS2_SP_ADDR);
    hide_data(GUN2_SP_ADDR);
    hide_data(AC_SP_ADDR);
}

void dwin_can_write(uint16_t addr, uint16_t value){
    write_display.buff[2] = (addr >> 8) & 0xFF;
    write_display.buff[3] = addr & 0xFF;
    write_display.buff[4] = (value >> 8) & 0xFF;
    write_display.buff[5] =value& 0xFF;

    can_network_transmit_std_id(TWAI_CAN, &write_display);
    ESP_LOGI(TAG, "Wrote value 0x%04X to address 0x%04X", value, addr);   
}

static void dwin_can_rx_handler(const can_msg_t *msg)
{
    ESP_LOGI(TAG,"Received id:%04X %02X %02X %02X %02X %02X %02X %02X %02X %d",msg->id,msg->buff[0],msg->buff[1],msg->buff[2],msg->buff[3],msg->buff[4],msg->buff[5],msg->buff[6],msg->buff[7],msg->dlc);
    
    if(msg->buff[0]==0x06 && msg->buff[1]==READ_CMD){
        if(msg->buff[2]==((CCS2_GUN1_S_STOP_ADDR>>8)&0xFF) &&msg->buff[3]==(CCS2_GUN1_S_STOP_ADDR&0xFF)){
            if(msg->buff[6]){ // START
                ESP_LOGI(TAG,"start command received");
                dwin_can_write(CCS2_GUN1_ADDR,STOP_CHARGING);
                remove_hide(CCS2_SP_ADDR,CCS2_ADDR_LOWER);
            }else{ // STOP
                ESP_LOGI(TAG,"stop command received");
                dwin_can_write(CCS2_GUN1_ADDR,START_CHARGING);
                hide_data(CCS2_SP_ADDR);
                }
            }
        else if(msg->buff[2]==((CCS2_GUN1_ADDR>>8)&0xFF)&&msg->buff[3]==(CCS2_GUN1_ADDR&0xFF)){
            ESP_LOGI(TAG,"CCS2 GUN STATUS READ");
            if(msg->buff[6]==START_CHARGING){
                hide_data(CCS2_SP_ADDR);
                dwin_can_write(CCS2_GUN1_S_STOP_ADDR,0);
                ESP_LOGI(TAG,"ccs2 gun hiding started");
            }
            else if(msg->buff[6]==PLUGGED){
                timer_addr_data = ((uint32_t)(CCS2_GUN1_ADDR & 0xFFFF) << 16) | (START_CHARGING & 0xFFFF);
                esp_timer_start_once(timer_3sec,2000000);
                ESP_LOGI(TAG,"ccs2 gun  timer started");
            }
        }
    }
}
static void clear_screen(uint16_t addr){
    dwin_can_write(ERR_STRING_ADDR,DISPLAY_TERMINATOR);
    dwin_can_write(ERR_NUMBER_ADDR,DISPLAY_TERMINATOR);
}

static void write_error_msg() {
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

static void dwin_init_pages(void){
    reset_display();
    display_event_loop_init();
    display_timer_event_loop_init();
    timer_addr_data=(uint32_t)(CCS2_GUN1_ADDR << 16 & 0xffFF) |(uint16_t)(UNPLUGGED & 0XFFFF);
    esp_timer_start_once(timer_3sec,3000000);//default screen after 3 sec
}

void dwin_can_write_float(uint16_t vp_addr, float value)
{   
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    uint16_t high = (raw >> 16) & 0xFFFF;
    uint16_t low  = raw & 0xFFFF;
    dwin_can_write(vp_addr,high);
    dwin_can_write(vp_addr+1,low);
    ESP_LOGI(TAG,"[TX-FLOAT] VP=0x%04X -> MSG->highx%2x lowx%2x and value %.3f\n", vp_addr,high,low, value);
}

void display_init(){
    can_network_register_rx_cb(TWAI_CAN, dwin_can_rx_handler);
    esp_timer_create(&ccs2_timer_args, &timer_3sec);
    dwin_init_pages();
}

