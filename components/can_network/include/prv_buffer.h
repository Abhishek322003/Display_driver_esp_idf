#include "esp_twai_types.h"

#define PRV_BUFFER_LEN (16U) //always a power of 2

typedef struct {
    twai_frame_t frame; //make sure twai_frame_t is always the first element
    uint8_t data[8];
}prv_buffer_item_t;

typedef struct
{
    uint32_t head;
    uint32_t len;
    prv_buffer_item_t items[PRV_BUFFER_LEN];
}prv_buffer_t;

static inline void prv_buffer_init(prv_buffer_t* buffer)
{
    buffer->head = 0;
    buffer->len = 0;
}

static inline void* prv_buffer_take(prv_buffer_t* buffer)
{
    if (buffer->len == PRV_BUFFER_LEN)
    {
        return NULL;
    }

    void* ret_ptr = &buffer->items[buffer->head];
    buffer->head = (buffer->head + 1) & (PRV_BUFFER_LEN - 1);
    buffer->len++;
    return ret_ptr;
    
}

static inline void prv_buffer_give(prv_buffer_t* buffer)
{   
    if (buffer->len != 0)
    {
       buffer->len--;
    }

}


