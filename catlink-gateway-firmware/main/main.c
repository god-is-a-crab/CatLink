#include "ble.h"
#include "lora.h"

void app_main(void) {
    lora_init();
    ble_init();
}
