idf_component_register(
    SRCS
        "can_network.c"
        "mcp2515_drv.c"

    INCLUDE_DIRS
        "include"

    PRIV_REQUIRES
        driver
        esp_ringbuf
        esp_system
        esp_timer
)
