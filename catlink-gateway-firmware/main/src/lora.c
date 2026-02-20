#include "lora.h"

#include "ble.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#define TAG "LoRa_Link"

#define SCAN_TIMEOUT            310

// SX1262 GPIO mapping for Heltec WiFi LoRa 32
#define GPIO_NUM_CS             8  // NSS
#define GPIO_NUM_CLK            9
#define GPIO_NUM_MOSI           10 // SPIQ
#define GPIO_NUM_MISO           11 // SPID
#define GPIO_NUM_NRST           12
#define GPIO_NUM_BUSY           13
#define GPIO_NUM_DIO1           14

#define SPI_HOST                SPI2_HOST // Use SPI2(FSPI)

// Buffer indexes
#define IRQ_MSB                 1
#define IRQ_LSB                 2
#define RX_BUF_STATUS_PL_LEN    1
#define WRITE_BUF_ADDR_LSB      3
#define READ_BUF_ADDR_B2        2
#define READ_BUF_ADDR_B1        3
#define READ_BUF_ADDR_B0        4
#define READ_BUF_PAYLOAD_START  5
#define PKT_STATUS_SNR          2
#define PKT_STATUS_SIGNAL_RSSI  3

// Packet masks
#define ADDR_LOWER_HALF_BYTE    ((uint8_t)(GATEWAY_ADDRESS << 4))
#define PID_MASK                0b00001100
#define BATTERY_FLAG            (1 << 0)
#define POSITION_FLAG           (1 << 1)

// DIO1 IRQ flags
#define TXDONE                  (1 << 0)
#define RXDONE                  (1 << 1)
#define CADDONE                 (1 << 7)
#define CADDETECTED             (1 << 0)
#define TIMEOUT                 (1 << 1)

typedef enum : uint8_t {
    LINK_DISCONNECTED = 0,
    LINK_SCAN_WAITING_TX,
    LINK_SCAN_WAITING_CAD,
    LINK_SCAN_WAITING_RX,
    LINK_CONNECTED_RX,
    LINK_CONNECTED_TX_ACK,
    LINK_CONNECTED_WAIT_RX,
} link_state_t;

static spi_device_handle_t spi;
static link_state_t link_state = LINK_DISCONNECTED;
static int scan_counter;
TaskHandle_t tracker_update_task_handle;
static TimerHandle_t rx_packet_timer;

static spi_transaction_t txn_set_sleep = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x84,
    .tx_data = {4},
    .length = 8 * 1,
};
static spi_transaction_t txn_set_standby_rc = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x80,
    .tx_data = {0},
    .length = 8 * 1,
};
static spi_transaction_t txn_set_packet_type_lora = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x8A,
    .tx_data = {1},
    .length = 8 * 1,
};
static spi_transaction_t txn_set_rf_frequency = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x86,
    .tx_data = {0x1b, 0x20, 0, 0}, // 434 Mhz
    .length = 8 * 4,
};
static spi_transaction_t txn_set_buffer_base_addr = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x8F,
    .tx_data = {0, 0x80},
    .length = 8 * 2,
};
static spi_transaction_t txn_set_mod_params = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x8B,
    // SF9, 125khz, 4-5 CR, low data rate optimization off
    .tx_data = {0x09, 0x04, 0x01, 0},
    .length = 8 * 4,
};
// Preamble length = 8, explicit header, 3 byte payload length, CRC off, standard IQ
static uint8_t set_packet_params_tx_data[6] = {0, 0x08, 0, 0x03, 0, 0};
static uint8_t set_packet_params_rx_data[6] = {0};
static spi_transaction_t txn_set_packet_params = {
    .cmd = 0x8C,
    .tx_buffer = set_packet_params_tx_data,
    .rx_buffer = set_packet_params_rx_data,
    .length = 8 * 6,
};
// Unmask TxDone, RxDone, Timeout, CadDone, CadDetected, map to DIO1
static uint8_t unmask_txdone_tx_data[8] = {0x00, 0x01, 0x00, 0x01, 0, 0, 0, 0};
static uint8_t unmask_txdone_rx_data[8] = {0};
static spi_transaction_t txn_unmask_txdone = {
    .cmd = 0x08,
    .tx_buffer = unmask_txdone_tx_data,
    .rx_buffer = unmask_txdone_rx_data,
    .length = 8 * 8,
};
static uint8_t unmask_rx_tx_data[8] = {0x02, 0x02, 0x02, 0x02, 0, 0, 0, 0};
static uint8_t unmask_rx_rx_data[8] = {0};
static spi_transaction_t txn_unmask_rx = {
    .cmd = 0x08,
    .tx_buffer = unmask_rx_tx_data,
    .rx_buffer = unmask_rx_rx_data,
    .length = 8 * 8,
};
static uint8_t unmask_cad_tx_data[8] = {0x01, 0x80, 0x00, 0x80, 0, 0, 0, 0};
static uint8_t unmask_cad_rx_data[8] = {0};
static spi_transaction_t txn_unmask_cad = {
    .cmd = 0x08,
    .tx_buffer = unmask_cad_tx_data,
    .rx_buffer = unmask_cad_rx_data,
    .length = 8 * 8,
};
static spi_transaction_t txn_write_sync_word = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x0D,
    // Sync word 0xAAF1
    .tx_data = {0x07, 0x40, 0xAA, 0xF1},
    .length = 8 * 4,
};
static spi_transaction_t txn_set_pa_config = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x95,
    .tx_data = {0x04, 0x07, 0x00, 0x01},
    .length = 8 * 4,
};
static spi_transaction_t txn_set_tx_params = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x8E,
    .tx_data = {0x16, 0x04},
    .length = 8 * 2,
};
static uint8_t set_cad_params_tx_data[7] = {0x02, 23, 10, 0, 0, 0, 0};
static uint8_t set_cad_params_rx_data[7] = {0};
static spi_transaction_t txn_set_cad_params = {
    .cmd = 0x88,
    .tx_buffer = set_cad_params_tx_data,
    .rx_buffer = set_cad_params_rx_data,
    .length = 8 * 7,
};
static spi_transaction_t txn_set_dio2_rf_switch_ctrl = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x9D,
    .tx_data = {1},
    .length = 8 * 1,
};
static spi_transaction_t txn_set_dio3_tcxo_ctrl = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x97,
    // TCXO voltage is 1.8V, wakeup delay set to 5ms
    // https://github.com/HelTecAutomation/Heltec_ESP32/blob/6551d566c5eab76869870b8dbf6885943c112683/src/driver/sx126x.c#L82
    // https://github.com/HelTecAutomation/Heltec_ESP32/blob/6551d566c5eab76869870b8dbf6885943c112683/src/driver/board-config.h#L39
    .tx_data = {0x02, 0, 0x01, 0x40},
    .length = 8 * 4,
};
static spi_transaction_t txn_write_buffer = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x0E,
    .tx_data = {0, (uint8_t)(GATEWAY_ADDRESS >> 12), (uint8_t)(GATEWAY_ADDRESS >> 4), ADDR_LOWER_HALF_BYTE},
    .length = 8 * 4,
};
// 1 byte read offset, 1 byte NOP, 3 bytes CatLink header, 1 byte battery block, 10 bytes position = 16 bytes
static uint8_t read_buffer_tx_data[16] = {0x80, 0};
static uint8_t read_buffer_rx_data[16] = {0};
static spi_transaction_t txn_read_buffer = {
    .cmd = 0x1E,
    .tx_buffer = read_buffer_tx_data,
    .rx_buffer = read_buffer_rx_data,
    .length = 8 * 16,
};
static spi_transaction_t txn_set_tx = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x83,
    // No TX timeout
    .tx_data = {0, 0, 0},
    .length = 8 * 3,
};
static spi_transaction_t txn_set_rx_scan = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x82,
    // 240 ms timeout
    .tx_data = {0, 0x3C, 0},
    .length = 8 * 3,
};
static spi_transaction_t txn_set_rx_first_packet = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x82,
    // 10 second timeout
    .tx_data = {0x09, 0xC4, 0},
    .length = 8 * 3,
};
static spi_transaction_t txn_set_rx_packet = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x82,
    // 2 second timeout
    .tx_data = {0x01, 0xF4, 0},
    .length = 8 * 3,
};
static spi_transaction_t txn_set_cad = {
    .cmd = 0xC5,
    .tx_data = {0},
    .length = 8 * 0,
};
static spi_transaction_t txn_clear_irq = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x02,
    .tx_data = {0x03, 0x83},
    .length = 8 * 2,
};
static spi_transaction_t txn_clear_tx_done = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x02,
    .tx_data = {0, 0x01},
    .length = 8 * 2,
};
static spi_transaction_t txn_clear_rx_done = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x02,
    .tx_data = {0x02, 0x02},
    .length = 8 * 2,
};
static spi_transaction_t txn_clear_cad = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x02,
    .tx_data = {0x01, 0x80},
    .length = 8 * 2,
};
static spi_transaction_t txn_get_irq_status = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x12,
    .tx_data = {0, 0, 0},
    .length = 8 * 3,
};
static spi_transaction_t txn_get_rx_buffer_status = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x13,
    .tx_data = {0, 0, 0},
    .length = 8 * 3,
};
static spi_transaction_t txn_get_packet_status = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .cmd = 0x14,
    .tx_data = {0, 0, 0, 0},
    .length = 8 * 4,
};

static void wait_for_busy_clear() {
    int busy;
    do {
        busy = gpio_get_level(GPIO_NUM_BUSY);
    } while (busy == 1);
}

// DIO1 handler
static void IRAM_ATTR dio1_isr_handler(void* arg) {
    BaseType_t xHigherPiorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(tracker_update_task_handle, DIO1_SET, eSetBits, &xHigherPiorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPiorityTaskWoken);
}

// Rx Packet Timer callback
static void rx_packet_timer_callback(TimerHandle_t xTimer) {
    xTaskNotify(tracker_update_task_handle, START_RX_PACKET, eSetBits);
}

inline static void send_spi_transaction(spi_transaction_t *txn, bool wait_busy) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_transmit(spi, txn));
    if (wait_busy) {
        wait_for_busy_clear();
    }
}

inline static void start_scan(link_state_t *link_state) {
    if (scan_counter > SCAN_TIMEOUT) {
        *link_state = LINK_DISCONNECTED;

        xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
        link_ctrl_val = CTRL_SCAN_TIMEDOUT;
        xSemaphoreGive(link_ctrl_mutex);
        send_link_ctrl_indication();

        ESP_LOGI(TAG, "Scan timed out");
        return;
    }
    // Transmit Advertisement
    send_spi_transaction(&txn_unmask_txdone, true);
    send_spi_transaction(&txn_set_tx, false);
    scan_counter++;
    *link_state = LINK_SCAN_WAITING_TX;
}

static void tracker_update_task(void *param) {
    uint32_t notify_val;

    // Initialize SX1262 transceiver
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
    wait_for_busy_clear(); // Wait for transceiver to start up
    send_spi_transaction(&txn_set_packet_type_lora, true);
    send_spi_transaction(&txn_set_rf_frequency, true);
    send_spi_transaction(&txn_set_buffer_base_addr, true);
    send_spi_transaction(&txn_set_mod_params, true);
    send_spi_transaction(&txn_set_packet_params, true);
    send_spi_transaction(&txn_write_sync_word, true);
    send_spi_transaction(&txn_set_pa_config, true);
    send_spi_transaction(&txn_set_tx_params, true);
    send_spi_transaction(&txn_set_cad_params, true);
    send_spi_transaction(&txn_set_dio2_rf_switch_ctrl, true);
    send_spi_transaction(&txn_set_dio3_tcxo_ctrl, true);
    send_spi_transaction(&txn_set_sleep, false);
    spi_device_release_bus(spi);

    while (1) {
        xTaskNotifyWait(0, ULONG_MAX, &notify_val, portMAX_DELAY);

        if ((notify_val & DIO1_SET) != 0 || (notify_val & START_RX_PACKET) != 0) {
            // Noop
        } else if ((notify_val & START_SCAN) != 0 && link_state == LINK_DISCONNECTED) {
            ESP_LOGI(TAG, "START SCAN");
            scan_counter = 0;
            send_spi_transaction(&txn_set_standby_rc, true); // Wake transceiver from sleep
            send_spi_transaction(&txn_write_buffer, true);
            start_scan(&link_state);
            continue;
        } else if ((notify_val & DISCONNECT) != 0 && link_state != LINK_DISCONNECTED) {
            ESP_LOGI(TAG, "DISCONNECT");
            link_state = LINK_DISCONNECTED;
        } else {
            continue;
        }

        switch (link_state) {
            case LINK_SCAN_WAITING_TX: {
                send_spi_transaction(&txn_clear_tx_done, true);

                // CAD for response
                send_spi_transaction(&txn_unmask_cad, true);
                send_spi_transaction(&txn_set_cad, false);
                link_state = LINK_SCAN_WAITING_CAD;
                break;
            }
            case LINK_SCAN_WAITING_CAD: {
                send_spi_transaction(&txn_get_irq_status, true);
                send_spi_transaction(&txn_clear_cad, true);
                if (txn_get_irq_status.rx_data[IRQ_MSB] & CADDETECTED) {
                    // CAD DETECTED, set RX for Syn/Ack
                    send_spi_transaction(&txn_unmask_rx, true);
                    send_spi_transaction(&txn_set_rx_scan, false);
                    link_state = LINK_SCAN_WAITING_RX;
                } else {
                    // CAD not detected, restart scan
                    start_scan(&link_state);
                }
                break;
            }
            case LINK_SCAN_WAITING_RX: {
                send_spi_transaction(&txn_get_irq_status, true);
                send_spi_transaction(&txn_clear_rx_done, true);
                if (txn_get_irq_status.rx_data[IRQ_LSB] & RXDONE) {
                    // RXDONE, connection established
                    send_spi_transaction(&txn_set_rx_first_packet, false);
                    link_state = LINK_CONNECTED_RX;

                    xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
                    link_ctrl_val = CTRL_CONNECTED;
                    xSemaphoreGive(link_ctrl_mutex);
                    send_link_ctrl_indication();
                    ESP_LOGI(TAG, "Connected");
                } else {
                    // TIMEOUT, restart scan
                    start_scan(&link_state);
                }
                break;
            }
            case LINK_CONNECTED_RX: {
                send_spi_transaction(&txn_get_irq_status, true);
                send_spi_transaction(&txn_clear_rx_done, true);

                if (txn_get_irq_status.rx_data[IRQ_LSB] & RXDONE) {
                    send_spi_transaction(&txn_get_rx_buffer_status, true);
                    const uint8_t payload_length = txn_get_rx_buffer_status.rx_data[RX_BUF_STATUS_PL_LEN];
                    txn_read_buffer.length = (payload_length + 2) * 8;
                    txn_read_buffer.rxlength = (payload_length + 2) * 8;
                    send_spi_transaction(&txn_read_buffer, true);
                    send_spi_transaction(&txn_get_packet_status, true);

                    // Transmit ACK
                    // TODO: write only necessary bytes using write offset, properly handle PID and TX power
                    txn_write_buffer.tx_data[WRITE_BUF_ADDR_LSB] = ADDR_LOWER_HALF_BYTE | (read_buffer_rx_data[READ_BUF_ADDR_B0] & PID_MASK);
                    send_spi_transaction(&txn_write_buffer, true);
                    send_spi_transaction(&txn_unmask_txdone, true);
                    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_start(spi, &txn_set_tx, portMAX_DELAY));

                    // Read packet fields
                    const uint32_t address = read_buffer_rx_data[READ_BUF_ADDR_B2] << 12
                                            | read_buffer_rx_data[READ_BUF_ADDR_B1] << 4
                                            | (read_buffer_rx_data[READ_BUF_ADDR_B0] & 0xF0) >> 4;
                    const uint8_t pid = (read_buffer_rx_data[READ_BUF_ADDR_B0] >> 2) & 3;
                    const uint8_t flags = read_buffer_rx_data[READ_BUF_ADDR_B0] & 3;

                    const int8_t snr = ((int8_t)txn_get_packet_status.rx_data[PKT_STATUS_SNR]) / 4;
                    const int8_t signal_rssi = (int8_t)(-(int16_t)txn_get_packet_status.rx_data[PKT_STATUS_SIGNAL_RSSI] / 2);

                    xSemaphoreTake(tracker_update_mutex, portMAX_DELAY);
                    tracker_update_chr_val[BLOCK_FLAGS_OFFSET] = flags;
                    tracker_update_chr_val[SNR_OFFSET] = snr;
                    tracker_update_chr_val[SIGNAL_RSSI_OFFSET] = signal_rssi;

                    ESP_LOGI(TAG, "Address: %lX, Pid: %u, Flags: %X, SNR: %d, RSSI: %d", address, pid, flags, snr, signal_rssi);

                    size_t block_idx = READ_BUF_PAYLOAD_START;

                    if (flags & BATTERY_FLAG) {
                        // Battery block
                        tracker_update_chr_val[BATTERY_OFFSET] = read_buffer_rx_data[block_idx];
                        ESP_LOGI(TAG, "Battery: %f", (float)read_buffer_rx_data[block_idx] / 255.0 * 6.0);
                        block_idx += 1;
                    }
                    if (flags & POSITION_FLAG) {
                        // Position block
                        memcpy(tracker_update_chr_val + POSITION_OFFSET, &read_buffer_rx_data[block_idx], 10);
                    }
                    xSemaphoreGive(tracker_update_mutex);
                    send_tracker_update_indication();

                    // Log position (debug only)
                    if (flags & POSITION_FLAG) {
                        ESP_LOGI(TAG, "Position: %X %X %X %X %X %X %X %X %X, hdop: %u",
                            read_buffer_rx_data[block_idx], read_buffer_rx_data[block_idx + 1], read_buffer_rx_data[block_idx + 2],
                            read_buffer_rx_data[block_idx + 3], read_buffer_rx_data[block_idx + 4], read_buffer_rx_data[block_idx + 5],
                            read_buffer_rx_data[block_idx + 6], read_buffer_rx_data[block_idx + 7], read_buffer_rx_data[block_idx + 8],
                            read_buffer_rx_data[block_idx + 9]);
                    }

                    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_end(spi, portMAX_DELAY));
                    link_state = LINK_CONNECTED_TX_ACK;
                } else {
                    // TIMEOUT
                    link_state = LINK_DISCONNECTED;
                    xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
                    link_ctrl_val = CTRL_CONNECTION_LOST;
                    xSemaphoreGive(link_ctrl_mutex);
                    send_link_ctrl_indication();
                    ESP_LOGI(TAG, "Tracker timed out, disconnected");
                }
                break;
            }
            case LINK_CONNECTED_TX_ACK: {
                send_spi_transaction(&txn_clear_tx_done, true);
                send_spi_transaction(&txn_unmask_rx, false);
                xTimerStart(rx_packet_timer, 0);
                link_state = LINK_CONNECTED_WAIT_RX;
                break;
            }
            case LINK_CONNECTED_WAIT_RX: {
                send_spi_transaction(&txn_set_rx_packet, false);
                link_state = LINK_CONNECTED_RX;
                break;
            }
            case LINK_DISCONNECTED: {
                xTimerStop(rx_packet_timer, 0);
                send_spi_transaction(&txn_set_standby_rc, true);
                send_spi_transaction(&txn_clear_irq, true);
                send_spi_transaction(&txn_set_sleep, false);
                esp_rom_delay_us(500);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void lora_init(void) {
    esp_err_t ret;

    // Configure Busy GPIO as input without interrupt
    gpio_config_t busy_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&busy_conf);

    // Configure DIO1 GPIO as input with interrupt
    gpio_config_t dio1_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_DIO1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // Interrupt on rising edge
    };
    gpio_config(&dio1_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // Attach ISR handler for DIO1
    gpio_isr_handler_add(GPIO_NUM_DIO1, dio1_isr_handler, NULL);

    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_MISO,
        .mosi_io_num = GPIO_NUM_MOSI,
        .sclk_io_num = GPIO_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = ESP_INTR_FLAG_IRAM, // ISR can be called if flash cache is disabled
        .max_transfer_sz = 32, // Max transfer size in bytes
    };

    // Initialize SPI bus
    ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);

    // Configure SPI device for SX1262
    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .address_bits = 0,       // No address phase
        .mode = 0,               // SPI Mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = SPI_MASTER_FREQ_8M,
        .spics_io_num = GPIO_NUM_CS,
        .queue_size = 32,         // Number of transactions queued
    };

    // Initialize device driver with SPI_HOST
    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    rx_packet_timer = xTimerCreate("RxPacketTimer", pdMS_TO_TICKS(8000), pdFALSE, NULL, rx_packet_timer_callback);

    xTaskCreate(tracker_update_task, "Tracker Update", 3*1024, NULL, 5, &tracker_update_task_handle);
}
