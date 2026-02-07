#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"

#define GATTS_TAG            "BTserver"
#define PROFILE_A_APP_ID     0
#define PROFILE_NUM          1
#define TEST_DEVICE_NAME     "ESP32_server"

#define NUMBER_OF_MOTORS     8
#define FLOAT_DATA_LEN       (sizeof(float) * NUMBER_OF_MOTORS)

// Default ATT MTU
#define DEFAULT_MTU_SIZE     23
static uint16_t mtu_size = DEFAULT_MTU_SIZE;

// 128-bit Service UUID
static const uint8_t GATTS_SERVICE_UUID_A[16] = {
    0xcb,0x34,0x1d,0xed, 0x25,0xf3, 0x24,0x4f,
    0x11,0xac, 0x7e,0xbc, 0x1a,0x24,0x7a,0x86
};
// 16-bit Characteristic UUIDs
#define GATTS_CHAR_UUID_TX   0xFF01
#define GATTS_CHAR_UUID_RX   0xFF02
#define GATTS_NUM_HANDLE_A   10

// Advertisement data (service UUID only)
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = false,
    .service_uuid_len    = 16,
    .p_service_uuid      = (uint8_t*)GATTS_SERVICE_UUID_A,
    .flag                = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// Scan response data (device name)
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
};
// Advertising parameters
static esp_ble_adv_params_t test_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Buffers for float data
static float tx_sensor_data[NUMBER_OF_MOTORS] = {0};
static float rx_buffer[NUMBER_OF_MOTORS]    = {0};

// Initial attribute values
static esp_attr_value_t gatts_initial_tx_char_val = {
    .attr_max_len = FLOAT_DATA_LEN,
    .attr_len     = FLOAT_DATA_LEN,
    .attr_value   = (uint8_t*)tx_sensor_data,
};
static esp_attr_value_t gatts_initial_rx_char_val = {
    .attr_max_len = FLOAT_DATA_LEN,
    .attr_len     = FLOAT_DATA_LEN,
    .attr_value   = (uint8_t*)rx_buffer,
};

// Connection and CCCD state
static bool is_connected = false;
static bool tx_notifications_enabled = false;
static uint16_t tx_descr_handle = 0;

// GATT profile instance structure
struct gatts_profile_inst {
    esp_gatts_cb_t     gatts_cb;
    uint16_t           gatts_if;
    uint16_t           conn_id;
    uint16_t           service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t           tx_char_handle;
    uint16_t           rx_char_handle;
};
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = { .gatts_cb = NULL, .gatts_if = ESP_GATT_IF_NONE },
};

// Prototypes
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

// Profile A event handler
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
                                          esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status=%d, app_id=%d",
                 param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id  = 0;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid128,
               GATTS_SERVICE_UUID_A, 16);
        esp_ble_gatts_create_service(gatts_if,
                                     &gl_profile_tab[PROFILE_A_APP_ID].service_id,
                                     GATTS_NUM_HANDLE_A);
        esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_config_adv_data(&scan_rsp_data);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status=%d, handle=%d",
                 param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(param->create.service_handle);

        // RX characteristic (WRITE | WRITE_NR)
        {
            esp_bt_uuid_t rx_uuid = { .len = ESP_UUID_LEN_16,
                                      .uuid.uuid16 = GATTS_CHAR_UUID_RX };
            esp_ble_gatts_add_char(
                param->create.service_handle,
                &rx_uuid,
                ESP_GATT_PERM_WRITE | ESP_GATT_PERM_WRITE_ENCRYPTED,
                ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                &gatts_initial_rx_char_val,
                NULL
            );
        }
        // TX characteristic (READ | NOTIFY)
        {
            esp_bt_uuid_t tx_uuid = { .len = ESP_UUID_LEN_16,
                                      .uuid.uuid16 = GATTS_CHAR_UUID_TX };
            esp_ble_gatts_add_char(
                param->create.service_handle,
                &tx_uuid,
                ESP_GATT_PERM_READ,
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                &gatts_initial_tx_char_val,
                NULL
            );
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_RX) {
            gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle = param->add_char.attr_handle;
        } else {
            gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle = param->add_char.attr_handle;
            // Client config descriptor
            esp_bt_uuid_t descr_uuid = { .len = ESP_UUID_LEN_16,
                                         .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG };
            esp_ble_gatts_add_char_descr(
                gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                &descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL,
                NULL
            );
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.descr_uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
            tx_descr_handle = param->add_char_descr.attr_handle;
        }
        break;

    case ESP_GATTS_MTU_EVT:
        mtu_size = param->mtu.mtu;
        ESP_LOGI(GATTS_TAG, "MTU updated: %d", mtu_size);
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client connected, conn_id=%d", param->connect.conn_id);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        is_connected = true;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client disconnected, restarting adv");
        is_connected = false;
        tx_notifications_enabled = false;
        esp_ble_gap_start_advertising(&test_adv_params);
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t rsp = {0};
        uint16_t offset = param->read.offset;
        size_t remain = FLOAT_DATA_LEN - offset;
        size_t chunk = (remain > (mtu_size - 1)) ? (mtu_size - 1) : remain;
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len    = chunk;
        memcpy(rsp.attr_value.value,
               ((uint8_t*)tx_sensor_data) + offset,
               chunk);
        esp_ble_gatts_send_response(
            gatts_if,
            param->read.conn_id,
            param->read.trans_id,
            ESP_GATT_OK,
            &rsp
        );
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        uint16_t offset = param->write.offset;
        size_t remain = FLOAT_DATA_LEN - offset;
        size_t chunk = (param->write.len < remain) ? param->write.len : remain;
        memcpy(((uint8_t*)rx_buffer) + offset,
               param->write.value,
               chunk);
        ESP_LOGI(GATTS_TAG, "Received %zu bytes at offset %d", chunk, offset);
        if (offset + chunk == FLOAT_DATA_LEN) {
            for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
                ESP_LOGI(GATTS_TAG, "rx[%d]=%f", i, rx_buffer[i]);
            }
        }
        if (param->write.handle == tx_descr_handle) {
            uint16_t cccd = param->write.value[1] << 8 | param->write.value[0];
            tx_notifications_enabled = (cccd == 0x0001);
            ESP_LOGI(GATTS_TAG, "Notifications %s",
                     tx_notifications_enabled ? "enabled" : "disabled");
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(
                gatts_if,
                param->write.conn_id,
                param->write.trans_id,
                ESP_GATT_OK,
                NULL
            );
        }
        break;
    }

    default:
        break;
    }
}

// GATT server event dispatcher
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT && param->reg.status == ESP_GATT_OK) {
        gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    }
    gatts_profile_a_event_handler(event, gatts_if, param);
}

// GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT ||
        event == ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&test_adv_params);
    }
}

// Initialization
void Bluetooth_Initialize(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_init(& (esp_bt_controller_config_t)BT_CONTROLLER_INIT_CONFIG_DEFAULT()));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_A_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(512));
}

// Notification task with fragmentation support
void ble_send_sensor_data_task(void *pvParameters) {
    while (1) {
        if (is_connected && tx_notifications_enabled) {
            size_t payload = mtu_size - 3;
            for (size_t offset = 0; offset < FLOAT_DATA_LEN; offset += payload) {
                size_t chunk = (FLOAT_DATA_LEN - offset > payload)
                                   ? payload
                                   : (FLOAT_DATA_LEN - offset);
                esp_ble_gatts_send_indicate(
                    gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
                    chunk,
                    (uint8_t*)tx_sensor_data + offset,
                    false
                );
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Main entry
void app_main(void) {
    Bluetooth_Initialize();
    xTaskCreatePinnedToCore(
        ble_send_sensor_data_task,
        "BLE_send_task",
        4096, NULL, 5, NULL, 0
    );
}
