#ifndef CAN_NETWORK_H
#define CAN_NETWORK_H

#include "esp_err.h"

#define CAN_MSG_BUFF_LEN 8U


typedef struct 
{  
    uint32_t id;
    uint8_t dlc;
    uint8_t buff[CAN_MSG_BUFF_LEN] __attribute__((aligned(8)));
}can_msg_t;


typedef enum
{
    TWAI_CAN = 0,
    SPI_CAN,
    CAN_DEV_MAX,
}can_dev_t;

typedef void (*can_rx_cb_t)(const can_msg_t* rx_msg);

void can_network_init(void);
esp_err_t can_network_transmit_std_id(const can_dev_t dev,const can_msg_t* tx_msg);
esp_err_t can_network_transmit_ext_id(const can_dev_t dev,const can_msg_t* tx_msg);
esp_err_t can_network_register_rx_cb(can_dev_t dev,can_rx_cb_t cb);



#endif /*CAN_NETWORK_H*/

