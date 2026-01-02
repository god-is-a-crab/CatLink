#ifndef LORA_H
#define LORA_H

#include "freertos/FreeRTOS.h"

#define GATEWAY_ADDRESS     0xD7B12UL

// Notification flags
#define DIO1_SET            (1 << 0)
#define DISCONNECT          (1 << 1)
#define START_SCAN          (1 << 2)
#define START_RX_PACKET     (1 << 3)


extern TaskHandle_t tracker_update_task_handle;

void lora_init(void);


#endif // LORA_H
