idf_component_register(
    SRCS "error.c" "energy_meter.c" "main.c" "display.c" "error.c"
    INCLUDE_DIRS "."
    REQUIRES esp-modbus nvs_flash driver esp_timer driver can_network esp_adc esp_event)