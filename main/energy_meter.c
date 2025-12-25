#include "energy_meter.h"
#include "esp_log.h"
#include "mbcontroller.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "display.h"
void *mb_ctx = 0;
#define TAG "ENERGY_METER"

static inline uint16_t float_to_u16(float v, float scale)
{
    if (v <= 0.0f) return 0;
    v *= scale;
    if (v > 65535.0f) return 65535;
    return (uint16_t)(v + 0.5f); 
}

static float decodeFloat(uint16_t low, uint16_t high)
{
    uint32_t raw = ((uint32_t)high << 16) | low;
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

static float read_float_id(uint8_t slave_id, uint16_t reg_start)
{
    mb_param_request_t req = {
        .slave_addr = slave_id,
        .command    = 0x03,
        .reg_start  = reg_start,
        .reg_size   = 2
    };
    uint16_t raw[2] = {0};
    esp_err_t err = mbc_master_send_request(mb_ctx, &req, raw);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Reg %u -> RAW: 0x%04X 0x%04X", reg_start, raw[1], raw[0]);
        return decodeFloat(raw[0], raw[1]);
    }
    ESP_LOGI(TAG, "Modbus Read Fail (reg %u), err=0x%x", reg_start, err);
    return -9999.0f;   
}

static void update_dc_meter(){
    dwin_can_write_float(CCS2_DEMAND_VOLTAGE ,read_float_id(DC_METER_SLAVE_ID,DC_REG_VOLT));
    dwin_can_write_float(CCS2_DEMAND_CURRENT ,read_float_id(DC_METER_SLAVE_ID,DC_REG_CURR));
    dwin_can_write_float(CCS2_ENERGY_DELIVERED,read_float_id(DC_METER_SLAVE_ID,DC_REG_KWH));
}

static void update_ac_meter(void)
{
    dwin_can_write_float(CCS2_VOLTAGE_V1 ,read_float_id(AC_METER_SLAVE_ID,AC_L1_VOL ));
    dwin_can_write_float(CCS2_VOLTAGE_V2 ,read_float_id(AC_METER_SLAVE_ID,AC_L2_VOL));
    dwin_can_write_float(CCS2_VOLTAGE_V3 ,read_float_id(AC_METER_SLAVE_ID,AC_L3_VOL));
    dwin_can_write_float(CCS2_AVG_VOLTAGE ,read_float_id(AC_METER_SLAVE_ID,AVG_LINE_VOL));

    dwin_can_write_float(CCS2_CURRENT_I1 ,read_float_id(AC_METER_SLAVE_ID,AC_L1_CUR ));
    dwin_can_write_float(CCS2_CURRENT_I2 ,read_float_id(AC_METER_SLAVE_ID,AC_L2_CUR));
    dwin_can_write_float(CCS2_CURRENT_I3 ,read_float_id(AC_METER_SLAVE_ID,AC_L3_CUR));
    dwin_can_write_float(CCS2_AVG_CURRENT ,read_float_id(AC_METER_SLAVE_ID,AVG_LINE_CUR));

    dwin_can_write_float(CCS2_ACTIVE_POWER  ,read_float_id(AC_METER_SLAVE_ID,AC_ACTIVE_POWER));
    dwin_can_write_float(CCS2_TOTAL_POWER ,read_float_id(AC_METER_SLAVE_ID,SYSTEM_PF));
    dwin_can_write_float(CCS2_FREQUENCY ,read_float_id(AC_METER_SLAVE_ID,SYSTEM_FREQUENCY));
    dwin_can_write_float(CCS2_AVG_PF ,read_float_id(AC_METER_SLAVE_ID,AC_L1_PF));
}

static void push_energy_meter_data(void *arg)
{
    while(1){
    update_ac_meter();
    update_dc_meter();
    vTaskDelay(100);
    }
}

static void write_random_data_to_display()
{
    uint16_t addr;
    for (addr = 0x1500; addr <= 0x1555; addr += 5){
        uint16_t random_data = rand() % 10000;  // random 0–9999
        //dwin_can_hide_write(addr,random_data);
        vTaskDelay(10);
    }
}

static void reset_energy_meter(void)
{
    mb_param_request_t req = {
        .command   = 0x06,   
        .reg_start = DC_REG_RST ,    
        .reg_size  = 1
    };
    uint16_t reset_value = METER_RST_VALUE;
    esp_err_t err;
    req.slave_addr = DC_METER_SLAVE_ID;
    err = mbc_master_send_request(mb_ctx, &req, &reset_value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "DC Meter kWh reset successful");
    } else {
        ESP_LOGE(TAG, "DC Meter reset failed, err=0x%x", err);
    }
    req.reg_start=AC_REG_RST;
    req.slave_addr = AC_METER_SLAVE_ID;
    err = mbc_master_send_request(mb_ctx, &req, &reset_value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AC Meter kWh reset successful");
    } else {
        ESP_LOGE(TAG, "AC Meter reset failed, err=0x%x", err);
    }
}

void energy_meter_init(void){
    nvs_flash_init();//modbus init
    gpio_set_direction((gpio_num_t)MODBUS_RE_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MODBUS_RE_DE_PIN, 0);  
    mb_communication_info_t comm = {};
    comm.mode = MB_PORT_SERIAL_MASTER;
    comm.ser_opts.port = MB_PORT_NUM;
    comm.ser_opts.baudrate = MODBUS_BAUDRATE;
    comm.ser_opts.parity = UART_PARITY_DISABLE;
    comm.ser_opts.data_bits = UART_DATA_8_BITS;
    comm.ser_opts.stop_bits = UART_STOP_BITS_1;
    ESP_ERROR_CHECK(mbc_master_create_serial(&comm, &mb_ctx));
    ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, MODBUS_TX_PIN, MODBUS_RX_PIN, MODBUS_RE_DE_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(mbc_master_start(mb_ctx));
    ESP_LOGI(TAG, "Modbus Master Started DC METER Slave ID 0x24 and AC METER SLAVE ID 10");
    xTaskCreate(push_energy_meter_data,"push_energy_meter",4096,NULL,7,NULL);
}
