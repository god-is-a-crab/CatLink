#include "ble.h"

#include "esp_sleep.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* NimBLE APIs */
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"

/* Defines */
#define DEVICE_NAME                     "CatLink"
#define NIMBLE_TAG                      "NimBLE_STACK"
#define GAP_TAG                         "NimBLE_GAP"
#define GATTS_TAG                       "NimBLE_GATTS"
#define GPIO_NUM_LED                    35
#define GPIO_NUM_ADC_CTRL               37
#define MAX_CONNECTIONS                 3
#define ADV_ITVL_MS                     500
#define ADV_TIMEOUT_MS                  240000
#define BLE_GAP_APPEARANCE              0x1441
#define BLE_GAP_LE_ROLE_PERIPHERAL      0x00

static TaskHandle_t led_task_handle;

/* Device BLE address */
static uint8_t addr_type;
static uint8_t addr_val[6] = {0};

/* CatLink service */
static const ble_uuid128_t catlink_svc_uuid = BLE_UUID128_INIT(
    0x20, 0x58, 0x53, 0x1C, 0x6B, 0xA5, 0xA9, 0x14,
    0xB9, 0x08, 0x25, 0x0E, 0x17, 0x5D, 0xA3, 0xCE,
);

// Char val mutexes
SemaphoreHandle_t tracker_update_mutex;
SemaphoreHandle_t link_ctrl_mutex;

/* Tracker update characteristic */
uint8_t tracker_update_chr_val[14] = {0};
static uint16_t tracker_update_chr_val_handle;
static const ble_uuid128_t tracker_chr_uuid = BLE_UUID128_INIT(
    0x21, 0x58, 0x53, 0x1C, 0x6B, 0xA5, 0xA9, 0x14,
    0xB9, 0x08, 0x25, 0x0E, 0x17, 0x5D, 0xA3, 0xCE,
);
static int tracker_update_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Link control characteristic */
link_ctrl_t link_ctrl_val = CTRL_DISCONNECT;
static uint16_t link_ctrl_chr_val_handle;
static const ble_uuid128_t link_ctrl_chr_uuid = BLE_UUID128_INIT(
    0x22, 0x58, 0x53, 0x1C, 0x6B, 0xA5, 0xA9, 0x14,
    0xB9, 0x08, 0x25, 0x0E, 0x17, 0x5D, 0xA3, 0xCE,
);
static int link_ctrl_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);

// Battery service
static const ble_uuid16_t battery_svc_uuid = BLE_UUID16_INIT(0x180F);
static const ble_uuid16_t battery_level_chr_uuid = BLE_UUID16_INIT(0x2A19);
static uint8_t battery_level_chr_val = 100;
static uint16_t battery_level_chr_val_handle;
static int battery_level_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);

static void start_advertising(void);

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* CatLink service */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &catlink_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { /* Tracker update characteristic */
                .uuid = &tracker_chr_uuid.u,
                .access_cb = tracker_update_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC |
                    BLE_GATT_CHR_F_INDICATE,
                .val_handle = &tracker_update_chr_val_handle
            },
            {
                .uuid = &link_ctrl_chr_uuid.u,
                .access_cb = link_ctrl_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC |
                    BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_INDICATE,
                .val_handle = &link_ctrl_chr_val_handle
            },
            {
                0, /* No more characteristics in this service. */
            }
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &battery_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &battery_level_chr_uuid.u,
                .access_cb = battery_level_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
                .val_handle = &battery_level_chr_val_handle
            },
            {
                0,
            }
        }
    },
    {
        0, /* No more services. */
    },
};

/* Connections */
typedef struct {
    bool conn_handle_inited;
    uint16_t conn_handle;
    uint8_t subscribed_chars;
} connection_t;
static connection_t connections[MAX_CONNECTIONS] = {0};

#define TRACKER_UPDATE_SUB_FLAG         (1 << 0)
#define LINK_CTRL_SUB_FLAG              (1 << 1)

void add_connection(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (!connections[i].conn_handle_inited) {
            connections[i].conn_handle_inited = true;
            connections[i].conn_handle = conn_handle;
            connections[i].subscribed_chars = 0;
            ESP_LOGI(GAP_TAG, "Added new connection in slot %d handle: %d", i, conn_handle);
            return;
        }
    }
    ESP_LOGW(GAP_TAG, "Max connections reached, cannot add new connection!");
}
void remove_connection(uint16_t conn_handle) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i].conn_handle_inited && connections[i].conn_handle == conn_handle) {
            connections[i].conn_handle_inited = false;
            ESP_LOGI(GAP_TAG, "Removed connection in slot %d handle: %d, subscribed_chars: %d", i, conn_handle, connections[i].subscribed_chars);
            return;
        }
    }
}
void subscribe_connection(uint16_t conn_handle, uint8_t char_sub_flag) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i].conn_handle_inited && connections[i].conn_handle == conn_handle) {
            connections[i].subscribed_chars |= char_sub_flag;
            ESP_LOGI(GAP_TAG, "Connection in slot %d handle: %d subscribed to char flag: %d, subscribed_chars: %d", i, conn_handle, char_sub_flag, connections[i].subscribed_chars);
            return;
        }
    }
}
void unsubscribe_connection(uint16_t conn_handle, uint8_t char_sub_flag) {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i].conn_handle_inited && connections[i].conn_handle == conn_handle) {
            connections[i].subscribed_chars &= ~char_sub_flag;
            ESP_LOGI(GAP_TAG, "Connection in slot %d handle: %d unsubscribed from char flag: %d, subscribed_chars: %d", i, conn_handle, char_sub_flag, connections[i].subscribed_chars);
            return;
        }
    }
}
bool no_connections() {
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i].conn_handle_inited) {
            ESP_LOGI(GAP_TAG, "Active connection in slot %d handle: %d", i, connections[i].conn_handle);
            return false;
        }
    }
    ESP_LOGI(GAP_TAG, "No active connections");
    return true;
}

bool is_connection_encrypted(uint16_t conn_handle) {
    struct ble_gap_conn_desc desc;

    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        ESP_LOGW(GAP_TAG, "failed to find connection by handle, error code: %d", rc);
        return false;
    }

    return desc.sec_state.encrypted;
}

void send_indication(uint8_t char_sub_flag) {
    uint16_t chr_val_handle;
    if (char_sub_flag == TRACKER_UPDATE_SUB_FLAG) {
        chr_val_handle = tracker_update_chr_val_handle;
    } else if (char_sub_flag == LINK_CTRL_SUB_FLAG) {
        chr_val_handle = link_ctrl_chr_val_handle;
    } else {
        ESP_LOGW(GATTS_TAG, "unknown characteristic flag for indication: %d", char_sub_flag);
        return;
    }
    // Send indications
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (connections[i].conn_handle_inited &&
            (connections[i].subscribed_chars & char_sub_flag) &&
            is_connection_encrypted(connections[i].conn_handle)) {
            ble_gatts_indicate(connections[i].conn_handle, chr_val_handle);
        }
    }
}
void send_tracker_update_indication(void) {
    send_indication(TRACKER_UPDATE_SUB_FLAG);
}
void send_link_ctrl_indication(void) {
    send_indication(LINK_CTRL_SUB_FLAG);
    xTaskNotify(led_task_handle, 1, eSetBits);
}

static int tracker_update_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Note: Tracker update characteristic is read only */
    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Verify attribute handle */
        if (attr_handle != tracker_update_chr_val_handle) {
            return BLE_ATT_ERR_ATTR_NOT_FOUND;
        }
        xSemaphoreTake(tracker_update_mutex, portMAX_DELAY);
        int rc = os_mbuf_append(ctxt->om, &tracker_update_chr_val, sizeof(tracker_update_chr_val));
        xSemaphoreGive(tracker_update_mutex);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    /* Unknown event */
    default:
        ESP_LOGW(GATTS_TAG, "unexpected access operation to tracker characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int link_ctrl_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        /* Verify attribute handle */
        if (attr_handle != link_ctrl_chr_val_handle) {
            return BLE_ATT_ERR_ATTR_NOT_FOUND;
        }
        if (ctxt->om->om_len != 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        link_ctrl_t val = ctxt->om->om_data[0];
        xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
        // Send link ctrl notification
        if ((link_ctrl_val == CTRL_SCAN || link_ctrl_val == CTRL_CONNECTED) && val == CTRL_DISCONNECT) {
            xTaskNotify(tracker_update_task_handle, DISCONNECT, eSetBits);
            link_ctrl_val = val;
        } else if ((link_ctrl_val == CTRL_DISCONNECT
                    || link_ctrl_val == CTRL_SCAN_TIMEDOUT
                    || link_ctrl_val == CTRL_CONNECTION_LOST)
                    && (val == CTRL_SCAN)) {
            xTaskNotify(tracker_update_task_handle, START_SCAN, eSetBits);
            link_ctrl_val = val;
        } else {
            xSemaphoreGive(link_ctrl_mutex);
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        xSemaphoreGive(link_ctrl_mutex);
        send_link_ctrl_indication();
        return 0;
    }
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        if (attr_handle != link_ctrl_chr_val_handle) {
            return BLE_ATT_ERR_ATTR_NOT_FOUND;
        }
        xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
        int rc = os_mbuf_append(ctxt->om, &link_ctrl_val, sizeof(link_ctrl_val));
        xSemaphoreGive(link_ctrl_mutex);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    /* Unknown event */
    default:
        ESP_LOGW(GATTS_TAG, "unexpected access operation to link ctrl characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int battery_level_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (attr_handle != battery_level_chr_val_handle) {
            return BLE_ATT_ERR_ATTR_NOT_FOUND;
        }
        int rc = os_mbuf_append(ctxt->om, &battery_level_chr_val, sizeof(battery_level_chr_val));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    default:
        ESP_LOGW(GATTS_TAG, "unexpected access operation to battery level characteristic, opcode: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/*
 *  GATT server subscribe event callback
 */
int gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    if (!is_connection_encrypted(event->subscribe.conn_handle)) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    uint8_t char_sub_flag;

    // Get char sub flag from attribute handle
    if (event->subscribe.attr_handle == tracker_update_chr_val_handle) {
        char_sub_flag = TRACKER_UPDATE_SUB_FLAG;
    } else if (event->subscribe.attr_handle == link_ctrl_chr_val_handle) {
        char_sub_flag = LINK_CTRL_SUB_FLAG;
    } else {
        return BLE_ATT_ERR_ATTR_NOT_FOUND;
    }
    if (event->subscribe.cur_indicate == 1) {
        subscribe_connection(event->subscribe.conn_handle, char_sub_flag);
    } else {
        unsubscribe_connection(event->subscribe.conn_handle, char_sub_flag);
    }
    return 0;
}

/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    int rc = 0;
    struct ble_gap_conn_desc desc;

    /* Handle different GAP event */
    switch (event->type) {

    /* Connect event */
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(GAP_TAG, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        /* Connection succeeded */
        if (event->connect.status == 0) {
            /* Check connection handle */
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGW(GAP_TAG, "failed to find connection by handle, error code: %d", rc);
                return rc;
            }

            add_connection(event->connect.conn_handle);

            /* Try to update connection parameters */
            struct ble_gap_upd_params params = {
                .itvl_min = desc.conn_itvl,
                .itvl_max = desc.conn_itvl,
                .latency = 3,
                .supervision_timeout = desc.supervision_timeout
            };
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0) {
                ESP_LOGW(GAP_TAG, "failed to update connection parameters, error code: %d", rc);
                return rc;
            }
        }
        /* Connection failed */
        else {
        }
        start_advertising();
        return rc;

    /* Disconnect event */
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated */
        remove_connection(event->disconnect.conn.conn_handle);
        if ((link_ctrl_val == CTRL_SCAN || link_ctrl_val == CTRL_CONNECTED) && no_connections()) {
            xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
            link_ctrl_val = CTRL_DISCONNECT;
            xSemaphoreGive(link_ctrl_mutex);
            xTaskNotify(tracker_update_task_handle, DISCONNECT, eSetBits);
            send_link_ctrl_indication();
        }
        ESP_LOGI(GAP_TAG, "disconnected from peer; handle=%d reason=%d", event->disconnect.conn.conn_handle, event->disconnect.reason);

        /* Restart advertising */
        if ((ble_gap_adv_active() == 1) && (ble_gap_adv_stop() != 0)) {
            ESP_LOGW(GAP_TAG, "failed to stop advertising");
        }
        start_advertising();
        return rc;

    /* Connection parameters update event */
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGD(GAP_TAG, "connection updated; status=%d", event->conn_update.status);

        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGW(GAP_TAG, "failed to find connection by handle, error code: %d", rc);
            return rc;
        }

        return rc;

    /* Advertising complete event */
    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* Advertising completed */
        ESP_LOGI(GAP_TAG, "advertise complete; reason=%d", event->adv_complete.reason);
        if (event->adv_complete.reason == BLE_HS_EPREEMPTED) {
            start_advertising();
        } else if (no_connections()) {
            esp_deep_sleep_start();
        }
        return rc;

    /* Notification sent event */
    case BLE_GAP_EVENT_NOTIFY_TX:
        if ((event->notify_tx.status != 0) &&
            (event->notify_tx.status != BLE_HS_EDONE)) {
            /* Print notification info on error */
            ESP_LOGW(GAP_TAG,
                     "notify event; conn_handle=%d attr_handle=%d "
                     "status=%d is_indication=%d",
                     event->notify_tx.conn_handle, event->notify_tx.attr_handle,
                     event->notify_tx.status, event->notify_tx.indication);
        }
        return rc;

    /* Subscribe event */
    case BLE_GAP_EVENT_SUBSCRIBE:
        /* Print subscription info to log */
        ESP_LOGD(GAP_TAG,
                 "subscribe event; conn_handle=%d attr_handle=%d "
                 "reason=%d prevn=%d curn=%d previ=%d curi=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.reason, event->subscribe.prev_notify,
                 event->subscribe.cur_notify, event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);

        /* GATT subscribe event callback */
        rc = gatt_svr_subscribe_cb(event);
        if (rc == BLE_ATT_ERR_INSUFFICIENT_AUTHEN) {
            /* Request connection encryption */
            return ble_gap_security_initiate(event->subscribe.conn_handle);
        }
        return rc;

    /* MTU update event */
    case BLE_GAP_EVENT_MTU:
        /* Print MTU update info to log */
        ESP_LOGI(GAP_TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.channel_id,
                 event->mtu.value);
        return rc;

    /* Encryption change event */
    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        if (event->enc_change.status == 0) {
            ESP_LOGI(GAP_TAG, "connection encrypted!");
        } else {
            ESP_LOGW(GAP_TAG, "connection encryption failed, status: %d", event->enc_change.status);
        }
        return rc;

    /* Repeat pairing event */
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Delete the old bond */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGW(GAP_TAG, "failed to find connection, error code %d", rc);
            return rc;
        }
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with pairing operation */
        ESP_LOGI(GAP_TAG, "repairing...");
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    /* Passkey action event */
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        /* Display action */
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            /* Generate passkey */
            struct ble_sm_io pkey = {0};
            pkey.action = event->passkey.params.action;
            pkey.passkey = GATEWAY_ADDRESS;
            ESP_LOGI(GAP_TAG, "enter passkey %" PRIu32 " on the peer side", pkey.passkey);
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            if (rc != 0) {
                ESP_LOGE(GAP_TAG, "failed to inject security manager io, error code: %d", rc);
                return rc;
            }
        }
        return rc;
    }
    return rc;
}

static const ble_uuid128_t adv_svc_uuids128[] = {
    catlink_svc_uuid,
};

/* Set advertising flags */
static const struct ble_hs_adv_fields adv_fields = {
    // Limited discoverability, BT classic not supported
    .flags = BLE_HS_ADV_F_DISC_LTD | BLE_HS_ADV_F_BREDR_UNSUP,
    .name = (uint8_t *)DEVICE_NAME,
    .name_len = sizeof(DEVICE_NAME) - 1,
    .name_is_complete = 1,
    .uuids128 = adv_svc_uuids128,
    .num_uuids128 = 1,
    .uuids128_is_complete = 1,
};

static struct ble_hs_adv_fields rsp_fields = {0};

static const struct ble_gap_adv_params adv_params = {
    /* Set undirected and limited discoverability */
    .conn_mode = BLE_GAP_CONN_MODE_UND,
    .disc_mode = BLE_GAP_DISC_MODE_LTD,
    /* Set advertising interval */
    .itvl_min = BLE_GAP_ADV_ITVL_MS(ADV_ITVL_MS),
    .itvl_max = BLE_GAP_ADV_ITVL_MS(ADV_ITVL_MS + 10),
};

static void start_advertising(void) {
    /* Start advertising */
    int rc = ble_gap_adv_start(addr_type, NULL, ADV_TIMEOUT_MS, &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(GAP_TAG, "advertising started!");
}

void adv_init(void) {
    int rc = 0;

    /* Make sure we have proper BT identity address set */
    rc = ble_hs_util_ensure_addr(1); // Prefer random address
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising */
    rc = ble_hs_id_infer_auto(1, &addr_type); // Request privacy
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "failed to infer address type, error code: %d", rc);
        return;
    }
    ESP_LOGI(GAP_TAG, "using address type %d", addr_type);

    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    /* Set device address */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = addr_type;
    rsp_fields.device_addr_is_present = 1;

    /* Set advertisement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "failed to set scan response data, error code: %d", rc);
        return;
    }

    start_advertising();
}

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(GATTS_TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(GATTS_TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(GATTS_TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

static void led_task(void *param) {
    gpio_set_level(GPIO_NUM_LED, 0);
    uint32_t notify_val;
    link_ctrl_t ctrl_val = CTRL_DISCONNECT;

    while (1) {
        notify_val = 0;
        xTaskNotifyWait(0, ULONG_MAX, &notify_val, pdMS_TO_TICKS(250));

        if (notify_val == 1) {
            xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
            ctrl_val = link_ctrl_val;
            xSemaphoreGive(link_ctrl_mutex);
        }

        notify_val = 0;
        gpio_set_level(GPIO_NUM_LED, 1);
        switch (ctrl_val) {
            case CTRL_DISCONNECT:
            case CTRL_SCAN_TIMEDOUT:
            case CTRL_CONNECTION_LOST:
                xTaskNotifyWait(0, ULONG_MAX, &notify_val, pdMS_TO_TICKS(1000));
                break;
            case CTRL_SCAN:
                xTaskNotifyWait(0, ULONG_MAX, &notify_val, pdMS_TO_TICKS(250));
                break;
            case CTRL_CONNECTED:
                continue;
        }
        gpio_set_level(GPIO_NUM_LED, 0);
        if (notify_val == 1) {
            xSemaphoreTake(link_ctrl_mutex, portMAX_DELAY);
            ctrl_val = link_ctrl_val;
            xSemaphoreGive(link_ctrl_mutex);
        }
    }
}

int gap_init(void) {
    int rc = 0;

    /* Call NimBLE GAP initialization API */
    ble_svc_gap_init();

    /* Set GAP device name */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "failed to set device name to %s, error code: %d", DEVICE_NAME, rc);
        return rc;
    }
    return rc;
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    int rc;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

/* Library function declarations */
void ble_store_config_init(void);

/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGW(NIMBLE_TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Security manager configuration */
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1; // Enable Secure Connections
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGD(NIMBLE_TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

static void get_vbat_voltage(void) {
    // Configure ADC unit
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc_oneshot_init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&adc_oneshot_init_config, &adc1_handle);

    // Configure channel
    adc_oneshot_chan_cfg_t adc_oneshot_chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &adc_oneshot_chan_cfg);

    // Calibration
    adc_cali_handle_t adc1_cali_handle;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);

    // Sample
    int raw;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw);

    // Convert to voltage
    int voltage_mv;
    adc_cali_raw_to_voltage(adc1_cali_handle, raw, &voltage_mv);

    // Get offset from 3300 mV, battery range is 3300 mV to 4100mV
    // Heltec WSL uses CE6260B33M which has 3.3V output - minimum voltage is around 3.4V
    int voltage_offset_3300 = voltage_mv - 3300;
    // Clamp to 0-800
    voltage_offset_3300 = voltage_offset_3300 < 0   ? 0 : voltage_offset_3300;
    voltage_offset_3300 = voltage_offset_3300 > 800 ? 800 : voltage_offset_3300;
    battery_level_chr_val = voltage_offset_3300 / 8;

    ESP_LOGI("ADC", "VBat voltage: %d mV, level: %d", voltage_mv, battery_level_chr_val);
    gpio_set_level(GPIO_NUM_ADC_CTRL, 1);
}

void ble_init(void) {
    int rc;
    esp_err_t ret;

    /* Create mutexes for thread-safe access */
    tracker_update_mutex = xSemaphoreCreateMutex();
    link_ctrl_mutex = xSemaphoreCreateMutex();
    if (tracker_update_mutex == NULL || link_ctrl_mutex == NULL) {
        ESP_LOGE(NIMBLE_TAG, "failed to create mutexes");
        return;
    }

    /*
     * NVS flash initialization
     * Dependency of BLE stack to store configurations
     */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(NIMBLE_TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(NIMBLE_TAG, "failed to initialize nimble stack, error code: %d ", ret);
        return;
    }

    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(GAP_TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }

    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(GATTS_TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    // Configure LED GPIO
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);

    // Configure ADC control GPIO
    gpio_config_t adc_ctrl_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_ADC_CTRL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&adc_ctrl_conf);
    gpio_set_level(GPIO_NUM_ADC_CTRL, 0); // Open pmos

    get_vbat_voltage();

    /* Print number of bonded devices */
    int bonded_count = 0;
    ble_addr_t peer_addr[MAX_CONNECTIONS] = {0};
    rc = ble_store_util_bonded_peers(peer_addr, &bonded_count, MAX_CONNECTIONS);
    if (rc != 0) {
        ESP_LOGW(GAP_TAG, "failed to get bonded devices, error code: %d", rc);
        return;
    }
    ESP_LOGI(GAP_TAG, "bonded devices count: %d", bonded_count);

    xTaskCreate(led_task, "LED Task", 3*1024, NULL, 4, &led_task_handle);

    /* Start NimBLE host task thread and return */
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);
    return;
}
