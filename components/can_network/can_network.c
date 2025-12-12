#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "can_network.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "mcp2515_drv.h"
#include "mcp2515_can.h"
#include "prv_buffer.h"


#define TAG "CAN Network"

//TWAI CAN Macros
#define TWAI_CAN_TX_PIN  GPIO_NUM_4
#define TWAI_CAN_RX_PIN   GPIO_NUM_5
#define TWAI_TX_QUEUE_DEPTH (PRV_BUFFER_LEN)
#define TWAI_CAN_BITRATE            125000


//SPI CAN Macros
#define SPI_CAN_CS_PIN   GPIO_NUM_10
#define SPI_CAN_CLK_PIN  GPIO_NUM_12
#define SPI_CAN_MISO_PIN GPIO_NUM_11
#define SPI_CAN_MOSI_PIN GPIO_NUM_13
#define SPI_CAN_INT_PIN GPIO_NUM_45
#define SPI_CAN_BITRATE CAN_250KBPS


#define TWAI_CAN_TX_BUF_ITEM_LEN (10U)
#define TWAI_CAN_TX_BUF_SIZE (size_t)(TWAI_CAN_TX_BUF_ITEM_LEN * sizeof(twai_can_tx_item_t))

#define TWAI_CAN_RX_BUF_ITEM_LEN (10U)
#define TWAI_CAN_RX_BUF_SIZE  (size_t)(TWAI_CAN_RX_BUF_ITEM_LEN * sizeof(can_msg_t))

#define SPI_CAN_TX_BUF_ITEM_LEN (20U)
#define SPI_CAN_TX_BUF_SIZE (size_t)(SPI_CAN_TX_BUF_ITEM_LEN * sizeof(can_msg_t))

#define SPI_CAN_RX_BUF_ITEM_LEN (20U)
#define SPI_CAN_RX_BUF_SIZE (size_t)(SPI_CAN_RX_BUF_ITEM_LEN * sizeof(can_msg_t))

#define SPI_CAN_EVT_TX_BIT (1)
#define SPI_CAN_EVT_INT_BIT (1 << 1)

#define CAN_RX_CBS_MAX (1U)

static QueueHandle_t spi_can_tx_buf_hdl = NULL;
static uint8_t spi_can_tx_arr[SPI_CAN_TX_BUF_SIZE];
static StaticQueue_t spi_can_tx_buf;

static QueueHandle_t spi_can_rx_buf_hdl = NULL;
static uint8_t spi_can_rx_arr[SPI_CAN_RX_BUF_SIZE];
static StaticQueue_t spi_can_rx_buf;

static TaskHandle_t spi_can_task_hdl;

static twai_node_handle_t inter_node = NULL;
static prv_buffer_t twai_can_tx_buff;
static portMUX_TYPE prv_buffer_spinlock = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t twai_can_rx_buf_hdl = NULL;
static uint8_t twai_can_rx_arr[TWAI_CAN_RX_BUF_SIZE];
static StaticQueue_t twai_can_rx_buf;

static TaskHandle_t can_rx_task_hdl;
static can_rx_cb_t can_rx_cb[CAN_DEV_MAX][CAN_RX_CBS_MAX];

static inline esp_err_t spi_can_transmit(void);
static inline esp_err_t spi_can_receive(RXBn_t rxbn);
static inline esp_err_t spi_can_tx_q_std_id(const can_msg_t* tx_msg);
static inline esp_err_t spi_can_tx_q_ext_id(can_msg_t* tx_msg);
static void process_spi_can_tx_ints(uint8_t tx_intr_val);
static void process_spi_can_rx_ints(uint8_t rx_intr_val);
static void process_spi_can_err_ints(uint8_t err_intr_val);
static void spi_can_task(void* param);
static void spi_can_init(void);

static inline esp_err_t twai_can_tx_std_id(const can_msg_t* tx_msg);
static inline esp_err_t twai_can_tx_ext_id(const can_msg_t* tx_msg);
static void twai_can_init(void);


static bool IRAM_ATTR twai_on_tx_cb(twai_node_handle_t node, const twai_tx_done_event_data_t *edata, void *user_ctx)
{   
    taskENTER_CRITICAL_ISR(&prv_buffer_spinlock);
    prv_buffer_give(&twai_can_tx_buff);
    taskEXIT_CRITICAL_ISR(&prv_buffer_spinlock);

    return false;
}

static bool IRAM_ATTR twai_on_rx_cb(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    twai_frame_t rx_frame = {0};
    can_msg_t rx_msg = {0};
    rx_frame.buffer = rx_msg.buff;
    rx_frame.buffer_len = CAN_MSG_BUFF_LEN;

    BaseType_t task_woken = pdFALSE;
    BaseType_t task_woken_1 = pdFALSE;


    if (twai_node_receive_from_isr(inter_node,&rx_frame) == ESP_OK)
    {
        rx_msg.dlc = rx_frame.header.dlc;
        rx_msg.id = rx_frame.header.id;

        xQueueSendToBackFromISR(twai_can_rx_buf_hdl,&rx_msg,&task_woken);
        vTaskNotifyGiveFromISR(can_rx_task_hdl,&task_woken_1);
    }
    
    return (task_woken == pdTRUE || task_woken_1 == pdTRUE);
}

static void IRAM_ATTR gpio_isr_handler(void* arg) 
{
    (void) arg;
    BaseType_t task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(spi_can_task_hdl,&task_woken);
    portYIELD_FROM_ISR(task_woken);

} 


static inline esp_err_t twai_can_tx_std_id(const can_msg_t* tx_msg)
{    
    prv_buffer_item_t* tx_item;

    taskENTER_CRITICAL(&prv_buffer_spinlock);
    tx_item = (prv_buffer_item_t*)prv_buffer_take(&twai_can_tx_buff);
    taskEXIT_CRITICAL(&prv_buffer_spinlock);
    
    if (!tx_item)
    {
        return ESP_ERR_NO_MEM;
    }
    
    (void) memcpy(tx_item->data,tx_msg->buff,CAN_MSG_BUFF_LEN);
    tx_item->frame.header.id = tx_msg->id;
    tx_item->frame.header.dlc = tx_msg->dlc;
    tx_item->frame.buffer = tx_item->data;
   
    ESP_ERROR_CHECK(twai_node_transmit(inter_node, &(tx_item->frame), 0));
    
    return ESP_OK;
}

static inline esp_err_t twai_can_tx_ext_id(const can_msg_t* tx_msg)
{
    
    prv_buffer_item_t* tx_item;

    taskENTER_CRITICAL(&prv_buffer_spinlock);
    tx_item = (prv_buffer_item_t*)prv_buffer_take(&twai_can_tx_buff);
    taskEXIT_CRITICAL(&prv_buffer_spinlock);
    
    if (!tx_item)
    {
        return ESP_ERR_NO_MEM;
    }
    
    (void) memcpy(tx_item->data,tx_msg->buff,CAN_MSG_BUFF_LEN);
    tx_item->frame.header.id = tx_msg->id;
    tx_item->frame.header.dlc = tx_msg->dlc;
    tx_item->frame.header.ide = true;
    tx_item->frame.buffer = tx_item->data;
   
    ESP_ERROR_CHECK(twai_node_transmit(inter_node, &(tx_item->frame), 0));
    
    return ESP_OK;

}


static inline esp_err_t spi_can_transmit(void)
{
    can_msg_t tx_msg;
    TXBn_t txbn;

    if (MCP2515_checkTXMailboxes(&txbn) != ERROR_OK)
    {
        return ESP_FAIL;
    }
    
    if(xQueueReceive(spi_can_tx_buf_hdl,&tx_msg,0) != pdTRUE)
    {   
        return ESP_FAIL;
    }

    
    if (MCP2515_sendMessage(txbn,(CAN_FRAME)&tx_msg) != ERROR_OK)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static inline esp_err_t spi_can_receive(RXBn_t rxbn)
{
    can_msg_t rx_msg;
    
    if (MCP2515_readMessage(rxbn, (CAN_FRAME)&rx_msg) != ERROR_OK) 
    {
        return ESP_FAIL;
    }

    rx_msg.id = rx_msg.id & (~(CAN_EFF_FLAG | CAN_RTR_FLAG));

    if (xQueueSendToBack(spi_can_rx_buf_hdl, &rx_msg,0) != pdTRUE)
    {
        return ESP_FAIL;
    }

    xTaskNotifyGive(can_rx_task_hdl);
    
    return ESP_OK;
               
}



static inline esp_err_t spi_can_tx_q_std_id(const can_msg_t* tx_msg)
{   
    
    esp_err_t err = ESP_OK;

    if (xQueueSendToBack(spi_can_tx_buf_hdl, tx_msg, 0) != pdTRUE)
    {
        err = ESP_ERR_NO_MEM;
    }

    xTaskNotifyGive(spi_can_task_hdl);
    return err;
    
}

static inline esp_err_t spi_can_tx_q_ext_id(can_msg_t* tx_msg)
{   
    esp_err_t err = ESP_OK;
    tx_msg->id |= CAN_EFF_FLAG; //set the extended id flag

    if (xQueueSendToBack(spi_can_tx_buf_hdl, tx_msg, 0) != pdTRUE)
    {
        err = ESP_ERR_NO_MEM;
    }

    xTaskNotifyGive(spi_can_task_hdl);
    return err;
}

static void process_spi_can_tx_ints(uint8_t tx_intr_val)
{
    tx_intr_val >>= 2; //check the CANINTF register's bitmap for understanding this
    
    while(tx_intr_val)
    {
        spi_can_transmit();
        tx_intr_val >>= 1;
    }

    MCP2515_clearTXInterrupts();
}

static void process_spi_can_rx_ints(uint8_t rx_intr_val)
{
    if (rx_intr_val & CANINTF_RX0IF)
    {
        spi_can_receive(RXB0);
    }

    if (rx_intr_val & CANINTF_RX1IF)
    {
        spi_can_receive(RXB1);
    }
}

static void process_spi_can_err_ints(uint8_t err_intr_val)
{
    if (!err_intr_val)
    {
        return;
    }
    
    uint8_t eflg = MCP2515_getErrorFlags();

    if ((eflg | EFLG_RX0OVR) == EFLG_RX0OVR || (eflg | EFLG_RX1OVR) == EFLG_RX1OVR)
    {
        ESP_LOGI(TAG,"SPI CAN ERROR: RX BUFFER OVERFLOW");
    }

    if ((eflg | EFLG_TXBO) == EFLG_TXBO)
    {
        ESP_LOGI(TAG,"SPI CAN ERROR: BUS OFF");
    }
}


static void spi_can_task(void* param)
{   
    (void) param;
    uint8_t intr_val;

    MCP2515_reset();
	MCP2515_setBitrate(CAN_250KBPS, MCP_8MHZ);
	MCP2515_setNormalMode();
    ulTaskNotifyTake(pdTRUE,0);
    
    while(1)
    {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

        spi_can_transmit();
        intr_val = MCP2515_getInterrupts();

        if (intr_val != 0)
        {
            process_spi_can_tx_ints((uint8_t)(intr_val & (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF)));
            process_spi_can_rx_ints((uint8_t)(intr_val & (CANINTF_RX0IF | CANINTF_RX1IF)));
            process_spi_can_err_ints((uint8_t)(intr_val & CANINTF_ERRIF));
            MCP2515_clearInterrupts();
        }

    }

    vTaskDelete(spi_can_task_hdl);

    
}


static void twai_can_init(void)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TWAI_CAN_TX_PIN,
            .rx = TWAI_CAN_RX_PIN,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = TWAI_CAN_BITRATE,
        },
        .fail_retry_cnt = -1,
        .tx_queue_depth = TWAI_TX_QUEUE_DEPTH,
        .flags = {
            .no_receive_rtr = true,
        },
    };

    twai_event_callbacks_t evt_cbs = {
        .on_tx_done = twai_on_tx_cb,
        .on_rx_done = twai_on_rx_cb,
    };


    twai_mask_filter_config_t filter_config = {
        .id = 0,
        .mask = 0,      
        .is_ext = true,    
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &inter_node));
    ESP_ERROR_CHECK(twai_node_config_mask_filter(inter_node, 0, &filter_config));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(inter_node,&evt_cbs,NULL));
    ESP_ERROR_CHECK(twai_node_enable(inter_node));

    prv_buffer_init(&twai_can_tx_buff);

    twai_can_rx_buf_hdl = xQueueCreateStatic(TWAI_CAN_RX_BUF_ITEM_LEN,sizeof(can_msg_t),twai_can_rx_arr,&twai_can_rx_buf);

    if (!twai_can_rx_buf_hdl)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }


}


static void spi_can_init(void)
{
    spi_bus_config_t bus_cfg = {
		.miso_io_num = SPI_CAN_MISO_PIN,
		.mosi_io_num = SPI_CAN_MOSI_PIN,
		.sclk_io_num = SPI_CAN_CLK_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0 // no limit
	};


	spi_device_interface_config_t dev_cfg = {
		.mode = 0, 
		.clock_speed_hz = 40000000, //TODO try increasing this 
		.spics_io_num = SPI_CAN_CS_PIN,
		.queue_size = 512
	};

    gpio_config_t intr_pin = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SPI_CAN_INT_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    MCP2515_init();
	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &MCP2515_Object->spi));

    gpio_config(&intr_pin);
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SPI_CAN_INT_PIN, gpio_isr_handler, NULL));


    spi_can_tx_buf_hdl = xQueueCreateStatic(SPI_CAN_TX_BUF_ITEM_LEN,sizeof(can_msg_t),spi_can_tx_arr,&spi_can_tx_buf);

    if (!spi_can_tx_buf_hdl)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    spi_can_rx_buf_hdl = xQueueCreateStatic(SPI_CAN_TX_BUF_ITEM_LEN,sizeof(can_msg_t),spi_can_rx_arr,&spi_can_rx_buf);

    if (!spi_can_rx_buf_hdl)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    if (xTaskCreatePinnedToCore(spi_can_task,"SPI_CAN_TASK",4096,NULL,3,&spi_can_task_hdl,tskNO_AFFINITY) != pdPASS)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

}

static void can_rx_task(void* param)
{   
    can_msg_t rx_msg;

    ulTaskNotifyTake(pdTRUE,0);
    while(1)
    {

        ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
        
         if(xQueueReceive(twai_can_rx_buf_hdl,&rx_msg,0) == pdTRUE)
        {
            for (uint8_t cb_n = 0; cb_n < CAN_RX_CBS_MAX; cb_n++)
            {
                 can_rx_cb[TWAI_CAN][cb_n](&rx_msg);
            }
     
        }

        if(xQueueReceive(spi_can_rx_buf_hdl,&rx_msg,0) == pdTRUE)
        {   
            for (uint8_t cb_n = 0; cb_n < CAN_RX_CBS_MAX; cb_n++)
            {
                 can_rx_cb[SPI_CAN][cb_n](&rx_msg);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));

    }

    vTaskDelete(can_rx_task_hdl);
}


void can_network_init(void)
{
    twai_can_init();
    spi_can_init();


    if (xTaskCreatePinnedToCore(can_rx_task,"CAN_RX_TASK",4096,NULL,3,&can_rx_task_hdl,tskNO_AFFINITY) != pdPASS)
    {
         ESP_ERROR_CHECK(ESP_FAIL);
    }
    

}

esp_err_t can_network_transmit_std_id(const can_dev_t dev,const can_msg_t* tx_msg)
{
    ESP_RETURN_ON_FALSE((dev < CAN_DEV_MAX), ESP_ERR_INVALID_ARG, TAG, "invalid argument: out of range value");
    ESP_RETURN_ON_FALSE(tx_msg, ESP_ERR_INVALID_ARG, TAG, "invalid argument: null argument");

    switch (dev)
    {
    case TWAI_CAN:
        return (twai_can_tx_std_id(tx_msg));
        
    case SPI_CAN:
        return (spi_can_tx_q_std_id(tx_msg));
    
    default:
        break;
    }

    return ESP_FAIL;

}

esp_err_t can_network_transmit_ext_id(const can_dev_t dev,const can_msg_t* tx_msg)
{
    ESP_RETURN_ON_FALSE((dev < CAN_DEV_MAX), ESP_ERR_INVALID_ARG, TAG, "invalid argument: out of range value");
    ESP_RETURN_ON_FALSE(tx_msg, ESP_ERR_INVALID_ARG, TAG, "invalid argument: null argument");

    switch (dev)
    {
    case TWAI_CAN:
        return (twai_can_tx_ext_id(tx_msg)); 
        
    case SPI_CAN:
        return (spi_can_tx_q_ext_id((can_msg_t*)tx_msg));
          
    default:
        break;
    }

    return ESP_FAIL;
 
}

esp_err_t can_network_register_rx_cb(can_dev_t dev,can_rx_cb_t cb)
{
    ESP_RETURN_ON_FALSE((dev < CAN_DEV_MAX), ESP_ERR_INVALID_ARG, TAG, "invalid argument: out of range value");
    ESP_RETURN_ON_FALSE(cb, ESP_ERR_INVALID_ARG, TAG, "invalid argument: null argument");
    
    for (uint8_t cb_n = 0; cb_n < CAN_RX_CBS_MAX; cb_n++)
    {
        can_rx_cb[dev][cb_n] = cb;
    }
    
    return ESP_OK;
}