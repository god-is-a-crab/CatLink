#ifndef BLE_H
#define BLE_H

#include "lora.h"

// Tracker update char field indexes
#define BLOCK_FLAGS_OFFSET      0
#define BATTERY_OFFSET          1
#define POSITION_OFFSET         2
#define SNR_OFFSET              12
#define SIGNAL_RSSI_OFFSET      13

/**
 * @brief The tracker update characteristic value.
 * ┌─────┬──────────────┬───────────────┬────────────────┬────────┬───────────────┐
 * │Byte │      0       │       1       │     2-11       │   12   │      13       │
 * ├─────┼──────────────┼───────────────┼────────────────┼────────┼───────────────┤
 * │Data │ Block Flags  │ Battery Block │ Position Block │ SnrPkt │ SignalRssiPkt │
 * └─────┴──────────────┴───────────────┴────────────────┴────────┴───────────────┘
 */
extern uint8_t tracker_update_chr_val[14];
extern SemaphoreHandle_t tracker_update_mutex;

/**
 * @brief Send indication for tracker update characteristic.
 */
void send_tracker_update_indication(void);

/**
 * @brief Link control enum.
 */
typedef enum : uint8_t {
    CTRL_DISCONNECT         = 0,
    CTRL_SCAN               = 1,
    CTRL_CONNECTED          = 2,
    CTRL_SCAN_TIMEDOUT      = 3,
    CTRL_CONNECTION_LOST    = 4,
} link_ctrl_t;

/**
 * @brief Tracker link control value.
 */
extern link_ctrl_t link_ctrl_val;
extern SemaphoreHandle_t link_ctrl_mutex;

/**
 * @brief Send indication for link control characteristic.
 */
void send_link_ctrl_indication(void);

/**
 * @brief Initialize GATT server and NimBLE stack.
 */
void ble_init(void);


#endif // BLE_H
