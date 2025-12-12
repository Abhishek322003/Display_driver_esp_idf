#include "energy_meter.h"
#include "esp_log.h"
#include "mbcontroller.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
void *mb_ctx = 0;
#define TAG "ENERGY_METER"


static float decodeFloat(uint16_t low, uint16_t high)
{
    uint32_t raw = ((uint32_t)high << 16) | low;
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}
float readFloatID(uint8_t slave_id, uint16_t reg_start)
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

    ESP_LOGE(TAG, "Modbus Read Fail (reg %u), err=0x%x", reg_start, err);

    return -9999.0f;   
}

void push_energy_meter_data()
{
    float voltage = readFloatID(DC_METER_SLAVE_ID, DC_REG_VOLT);
    float current = readFloatID(DC_METER_SLAVE_ID, DC_REG_CURR);
    float power = readFloatID(DC_METER_SLAVE_ID, DC_REG_WATT);
    float energy = readFloatID(DC_METER_SLAVE_ID, DC_REG_KWH);

    ESP_LOGI(TAG, "DC Meter - Voltage: %.2f V, Current: %.2f A, Power: %.2f W, Energy: %.2f kWh",
             voltage, current, power, energy);
}   


void energy_meter_init(void){
     nvs_flash_init();
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
    ESP_LOGI(TAG, "Modbus Master Started (DC METER Slave ID 0x24)");
   
}
