#include "ble.h"
#include "cpg.h"

/* Default PID values when mobile app is used (mobile sends no Kp/Ki/Kd) */
#define MOBILE_DEFAULT_KP  0.25f
#define MOBILE_DEFAULT_KI  1.0f
#define MOBILE_DEFAULT_KD  0.002f
#define MOBILE_DEFAULT_F4  0.0f
#define MOBILE_DEFAULT_F5  0.0f

// ============================ MOBILE APP CHAR-TO-BYTE MAPPING ============================
// Single-char from mobile app -> control byte. Mobile sends one char at a time.
// Mode: H=Homing, I=Idle, S=Standby, D=Crawl, W=Walk, G=Gallop, T=Trot, C=Creep
// Orient: N=Normal, E=Inverted  |  Pivot: P=Trot, Q=Crawl
// Turning: 1/2=Trot L/R, 3/4=Creep L/R, 5/6=Crawl L/R, 7/8=Walk L/R, 9/0=Gallop L/R
// Posture: U=Normal, L=Low, O=Crouch
static uint8_t mobile_char_to_byte(char c) {
    switch (c) {
        case 'H': case 'h': return CPG_MODE_HOMING;       // Homing
        case 'I': case 'i': return CPG_MODE_IDLE;         // Idle
        case 'S': case 's': return CPG_MODE_STANDBY;       // Standby
        case 'D': case 'd': return MODE_CRAWL;             // Crawl
        case 'W': case 'w': return MODE_WALK;              // Walk
        case 'G': case 'g': return MODE_GALOP;            // Gallop
        case 'T': case 't': return MODE_TURTLE;           // Trot
        case 'C': case 'c': return MODE_CREEP;            // Creep
        case 'N': case 'n': return LEG_ORIENTATION_NORMAL;   // Leg normal
        case 'E': case 'e': return LEG_ORIENTATION_INVERTED;  // Leg inverted
        case 'P': case 'p': return MODE_PIVOT_TURN;       // Pivot trot
        case 'Q': case 'q': return MODE_PIVOT_CRAWL;      // Pivot crawl
        case '1': return MODE_TROT_LEFT;     case '2': return MODE_TROT_RIGHT;
        case '3': return MODE_CREEP_LEFT;    case '4': return MODE_CREEP_RIGHT;
        case '5': return MODE_CRAWL_LEFT;    case '6': return MODE_CRAWL_RIGHT;
        case '7': return MODE_WALK_LEFT;     case '8': return MODE_WALK_RIGHT;
        case '9': return MODE_GALOP_LEFT;    case '0': return MODE_GALOP_RIGHT;
        case 'U': case 'u': return BODY_POSTURE_NORMAL;    // Posture normal
        case 'L': case 'l': return BODY_POSTURE_LOW;       // Posture low
        case 'O': case 'o': return BODY_POSTURE_CROUCH;    // Posture crouch
        default: return 0xFF;
    }
}

// ============================ GLOBAL VARIABLES ============================
// Mutex and synchronization
portMUX_TYPE ble_mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t pid_params_mutex = NULL;
static SemaphoreHandle_t ble_data_mutex;
// BLE state management
static bool is_connected = false;
static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)
// Data buffers
volatile DRAM_ATTR uint8_t byte_val[NUMBER_OF_BYTES];
volatile DRAM_ATTR float float_val[NUMBER_OF_FLOATS];
PIDParams_t pid_ble_params;
// BLE communication
static QueueHandle_t BLE_recieve_queue;
static esp_gatt_status_t status;
static prepare_type_env_t prepare_write_env;
TaskHandle_t rx_task_handle;
// Declare all helper functions before they're used
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void handle_reg_evt(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void handle_create_evt(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void handle_add_char_evt(esp_ble_gatts_cb_param_t *param);
static void handle_add_char_descr_evt(esp_ble_gatts_cb_param_t *param);
static void handle_connect_evt(esp_ble_gatts_cb_param_t *param);
static void handle_write_evt(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void handle_immediate_write(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void handle_cccd_write(esp_ble_gatts_cb_param_t *param);
static void send_write_response(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void handle_exec_write_evt(esp_ble_gatts_cb_param_t *param);
static void handle_disconnect_evt(esp_ble_gatts_cb_param_t *param);
// Profile instance
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};
// UUID definitions
static const uint8_t GATTS_SERVICE_UUID_A[16] = {
    0xcb, 0x34, 0x1d, 0xed, 0x25, 0xf3, 0x24, 0x4f,
    0x11, 0xac, 0x7e, 0xbc, 0x1a, 0x24, 0x7a, 0x86
};
// Characteristic properties
esp_gatt_char_prop_t a_tx_property, a_rx_property;
// Characteristic values
esp_attr_value_t gatts_initial_tx_char_val = {
    .attr_max_len = NUM_MOTORS*sizeof(float),
    .attr_len = 0,
    .attr_value = NULL,
};
esp_attr_value_t gatts_initial_rx_char_val = {
    .attr_max_len = SERVER_MAX_DATA_LEN,
    .attr_len = 0,
    .attr_value = NULL,
};
// BLE configuration
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t *)GATTS_SERVICE_UUID_A,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
static esp_ble_adv_params_t test_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
// ============================ UTILITY FUNCTIONS ============================
bool validate_rx_packet(const ble_data_packet_t *packet) {
    if (packet->length == 0 || packet->length > SERVER_MAX_DATA_LEN) {
        ESP_LOGE(BLE_TAG, "Invalid packet length: %zu", packet->length);
        return false;
    }
    return true;
}
// ============================ BLE EVENT HANDLERS ============================
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (param->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_TAG, "Advertising start failed");
            } else if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&test_adv_params);
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(BLE_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        default:
            ESP_LOGI(BLE_TAG, "Unhandled GAP event: %d", event);
            break;
    }
}
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(BLE_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    // Route events to appropriate profile
    for (int idx = 0; idx < PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
            if (gl_profile_tab[idx].gatts_cb) {
                gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}
// ============================ WRITE EVENT MANAGEMENT ============================
void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    status = ESP_GATT_OK;
   
    if (param->write.is_prep) {
        if (param->write.offset > SERVER_MAX_DATA_LEN) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > SERVER_MAX_DATA_LEN) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
        if (status == ESP_GATT_OK) {
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(SERVER_MAX_DATA_LEN * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(BLE_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }
           
            if (xSemaphoreTake(ble_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                memcpy(prepare_write_env->prepare_buf + param->write.offset,
                       param->write.value,
                       param->write.len);
                prepare_write_env->prepare_len += param->write.len;
                xSemaphoreGive(ble_data_mutex);
            }
        }
    }
}
void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
        ESP_LOGW(BLE_TAG, "ESP_GATT_PREP_WRITE_EXEC pass data");
        if (xSemaphoreTake(ble_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (prepare_write_env->prepare_len > 0) {
                ble_data_packet_t packet;
                memcpy(packet.data, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
                packet.length = prepare_write_env->prepare_len;
               
                if (xQueueSend(BLE_recieve_queue, &packet, pdMS_TO_TICKS(10)) != pdPASS) {
                    ESP_LOGE(BLE_TAG, "Queue full - failed to pass data");
                }
            }
            xSemaphoreGive(ble_data_mutex);
        }
    }
   
    // Cleanup
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}
// ============================ PROFILE EVENT SUB-HANDLERS ============================
static void handle_reg_evt(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(BLE_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
   
    // Configure service
    gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
    gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
    memcpy(gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid128, GATTS_SERVICE_UUID_A, 16);
    esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_A);
    esp_ble_gap_set_device_name(DEVICE_NAME);
    // Configure advertising data
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
        ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
    }
    adv_config_done |= adv_config_flag;
}
static void handle_create_evt(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(BLE_TAG, "CREATE_SERVICE_EVT, status %d, service_handle %" PRIu16,
             param->create.status, param->create.service_handle);
            
    gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
    // Setup characteristics UUIDs
    gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TX;
   
    gl_profile_tab[PROFILE_A_APP_ID].rx_char_uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_A_APP_ID].rx_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_RX;
    // Start service
    esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
    // Add RX characteristic (Write)
    a_rx_property = ESP_GATT_CHAR_PROP_BIT_WRITE;
    esp_err_t add_rx_char_ret = esp_ble_gatts_add_char(
        gl_profile_tab[PROFILE_A_APP_ID].service_handle,
        &gl_profile_tab[PROFILE_A_APP_ID].rx_char_uuid,
        ESP_GATT_PERM_WRITE,
        a_rx_property,
        &gatts_initial_rx_char_val,
        NULL);
    if (add_rx_char_ret) {
        ESP_LOGE(BLE_TAG, "add rx_char failed, error code =%x", add_rx_char_ret);
    }
    // Add TX characteristic (Read + Notify)
    a_tx_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    esp_err_t add_tx_char_ret = esp_ble_gatts_add_char(
        gl_profile_tab[PROFILE_A_APP_ID].service_handle,
        &gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid,
        ESP_GATT_PERM_READ,
        a_tx_property,
        &gatts_initial_tx_char_val,
        NULL);
    if (add_tx_char_ret) {
        ESP_LOGE(BLE_TAG, "add tx_char failed, error code =%x", add_tx_char_ret);
    }
}
static void handle_add_char_evt(esp_ble_gatts_cb_param_t *param) {
    uint16_t length = 0;
    const uint8_t *prf_char;
   
    esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
    if (get_attr_ret == ESP_FAIL) {
        ESP_LOGE(BLE_TAG, "ILLEGAL HANDLE");
    }
    if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_TX) {
        ESP_LOGI(BLE_TAG, "ADD_TX_CHAR, status %d, attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
       
        // Add CCCD descriptor for TX characteristic
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
            gl_profile_tab[PROFILE_A_APP_ID].service_handle,
            &gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL, NULL);
        if (add_descr_ret) {
            ESP_LOGE(BLE_TAG, "add tx_char descr failed, error code = %x", add_descr_ret);
        }
    }
    else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_RX) {
        ESP_LOGI(BLE_TAG, "ADD_RX_CHAR, status %d, attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle = param->add_char.attr_handle;
    }
}
static void handle_add_char_descr_evt(esp_ble_gatts_cb_param_t *param) {
    if (gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid.uuid.uuid16 == param->add_char_descr.descr_uuid.uuid.uuid16) {
        gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle = param->add_char_descr.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value = 0x000;
        ESP_LOGI(BLE_TAG, "TX CCCD Handle: %d", gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle);
    }
}
static void handle_connect_evt(esp_ble_gatts_cb_param_t *param) {
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
   
    conn_params.latency = 0;
    conn_params.max_int = 0x30; // 40ms
    conn_params.min_int = 0x10; // 20ms
    conn_params.timeout = 600; // 4000ms
    ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x, link_role %d",
             param->connect.conn_id,
             param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
             param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
             param->connect.link_role);
    gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
    esp_ble_gap_update_conn_params(&conn_params);
    is_connected = true;
}
static void handle_write_evt(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, handle %"PRIu16, param->write.handle);
    if (!param->write.is_prep) {
        handle_immediate_write(gatts_if, param);
    }
   
    write_event_env(gatts_if, &prepare_write_env, param);
    send_write_response(gatts_if, param);
}
static inline void handle_immediate_write(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    size_t len = param->write.len;
    ESP_LOGI(BLE_TAG, "BLE data receiving length: %zu/%d", len, SERVER_MAX_DATA_LEN);
    if (param->write.handle == gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle) {
        ble_data_packet_t packet;
        packet.length = len;
        memcpy(packet.data, param->write.value, len);
        if (validate_rx_packet(&packet)) {
            if (xQueueSend(BLE_recieve_queue, &packet, pdMS_TO_TICKS(10)) != pdPASS) {
                ESP_LOGE(BLE_TAG, "Queue full - packet dropped");
            }
        } else {
            ESP_LOGE(BLE_TAG, "Invalid packet - dropped");
        }
    }
    else if (param->write.handle == gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle && len == 2) {
        handle_cccd_write(param);
    }
}
static void handle_cccd_write(esp_ble_gatts_cb_param_t *param) {
    uint16_t cccd_value = param->write.value[1] << 8 | param->write.value[0];
    gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value = cccd_value;
    ESP_LOGI(BLE_TAG, "CCCD Write: %04x", cccd_value);
    if (cccd_value == 0x0001) {
        if (a_tx_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
            ESP_LOGI(BLE_TAG, "notify enable");
            gl_profile_tab[PROFILE_A_APP_ID].tx_notifications_enabled = true;
        }
    } else {
        ESP_LOGI(BLE_TAG, "Notifications disabled");
        gl_profile_tab[PROFILE_A_APP_ID].tx_notifications_enabled = false;
    }
}
static inline void send_write_response(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (param->write.need_rsp) {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp) {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                                param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        } else {
            ESP_LOGE(BLE_TAG, "malloc failed, no resource to send response");
            status = ESP_GATT_NO_RESOURCES;
        }
    }
}
static inline void handle_exec_write_evt(esp_ble_gatts_cb_param_t *param) {
    exec_write_event_env(&prepare_write_env, param);
}
static void handle_disconnect_evt(esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(BLE_TAG, "CLIENT DISCONNECTED, conn_id %d, reason %d",
             param->disconnect.conn_id, param->disconnect.reason);
            
    gl_profile_tab[PROFILE_A_APP_ID].conn_id = 0;
    // Clear queue
    xQueueReset(BLE_recieve_queue);
    // Restart advertising
    esp_ble_gap_start_advertising(&test_adv_params);
    is_connected = false;
}
// ============================ DATA PROCESSING TASK ============================
// Handles both: (1) PC BLE GUI full packets [0xA5, ctrl, floats...], (2) Mobile app single-char commands
void copy_ble_recieve_data_task(void *pvParameters) {
    uint8_t log_counter = 0;
    static ble_data_packet_t local_rx_buffer;

    while (1) {
        if (xQueueReceive(BLE_recieve_queue, &local_rx_buffer, portMAX_DELAY) == pdTRUE) {
            if (!validate_rx_packet(&local_rx_buffer)) {
                ESP_LOGE(BLE_TAG, "Queue contained invalid packet - skipping");
                continue;
            }

            size_t len = local_rx_buffer.length;
            uint8_t *data = local_rx_buffer.data;

            /* --- Mobile app: single character (or short string) --- */
            if (len <= 3 && (len == 0 || data[0] != PACKET_START_MARKER)) {
                char c = (len > 0) ? (char)data[0] : '\0';
                uint8_t cmd = mobile_char_to_byte(c);
                if (cmd != 0xFF) {
                    taskENTER_CRITICAL(&ble_mux);
                    byte_val[0] = cmd;
                    /* Mobile app sends no Kp/Ki/Kd - use defaults */
                    float_val[0] = MOBILE_DEFAULT_KP;
                    float_val[1] = MOBILE_DEFAULT_KI;
                    float_val[2] = MOBILE_DEFAULT_KD;
                    float_val[3] = MOBILE_DEFAULT_F4;
                    float_val[4] = MOBILE_DEFAULT_F5;
                    pid_ble_params.Kp = MOBILE_DEFAULT_KP;
                    pid_ble_params.Ki = MOBILE_DEFAULT_KI;
                    pid_ble_params.Kd = MOBILE_DEFAULT_KD;
                    taskEXIT_CRITICAL(&ble_mux);
                    ESP_LOGI(BLE_TAG, "Mobile cmd: '%c' -> 0x%02X (Kp=%.2f Ki=%.2f Kd=%.3f)", c, cmd, MOBILE_DEFAULT_KP, MOBILE_DEFAULT_KI, MOBILE_DEFAULT_KD);
                } else if (len > 0) {
                    ESP_LOGW(BLE_TAG, "Unknown mobile char: '%c' (0x%02X)", c, (unsigned)data[0]);
                }
            }
            /* --- PC BLE GUI: full packet [0xA5, ctrl_byte, 5 floats, 0xAA, 0x5A] --- */
            else if (len >= 2 && data[0] == PACKET_START_MARKER) {
                int offset = 1;
                taskENTER_CRITICAL(&ble_mux);
                memcpy((void*)byte_val, (const void*)(data + offset), NUMBER_OF_BYTES);
                offset += NUMBER_OF_BYTES;
                if (len >= (size_t)(2 + NUMBER_OF_BYTES + NUMBER_OF_FLOATS * (int)sizeof(float))) {
                    memcpy((void*)float_val, (const void*)(data + offset), NUMBER_OF_FLOATS * sizeof(float));
                    pid_ble_params.Kp = float_val[0];
                    pid_ble_params.Ki = float_val[1];
                    pid_ble_params.Kd = float_val[2];
                }
                taskEXIT_CRITICAL(&ble_mux);
            }

            log_counter++;
            if (log_counter >= 1) {
                for (int i = 0; i < NUMBER_OF_BYTES; i++) {
                    ESP_LOGW(BLE_TAG, "Byte[%d] = %u (0x%02X)", i, byte_val[i], byte_val[i]);
                }
                for (int i = 0; i < 5; i++) {
                    ESP_LOGW(BLE_TAG, "Float[%d] = %.5f", i, float_val[i]);
                }
                ESP_LOGW(BLE_TAG, "BLE rx executed, length=%zu", len);
                log_counter = 0;
            }
        }

        memset(&local_rx_buffer, 0, sizeof(local_rx_buffer));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================ PROFILE A EVENT HANDLER ============================
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            handle_reg_evt(gatts_if, param);
            break;
           
        case ESP_GATTS_CREATE_EVT:
            handle_create_evt(gatts_if, param);
            break;
           
        case ESP_GATTS_ADD_CHAR_EVT:
            handle_add_char_evt(param);
            break;
           
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            handle_add_char_descr_evt(param);
            break;
           
        case ESP_GATTS_CONNECT_EVT:
            handle_connect_evt(param);
            break;
           
        case ESP_GATTS_WRITE_EVT:
            handle_write_evt(gatts_if, param);
            break;
           
        case ESP_GATTS_EXEC_WRITE_EVT:
            handle_exec_write_evt(param);
            break;
           
        case ESP_GATTS_DISCONNECT_EVT:
            handle_disconnect_evt(param);
            break;
           
        case ESP_GATTS_RESPONSE_EVT:
            ESP_LOGI(BLE_TAG, "GATT_RESPONSE_EVT: status %d", param->rsp.status);
            break;
           
        default:
            ESP_LOGI(BLE_TAG, "Unhandled GATTS event: %d", event);
            break;
    }
}
// ============================ BLE INITIALIZATION ============================
void Bluetooth_Initialize()
{
    esp_err_t ret;
    // Initialize NVS.
   // nvs_flash_erase();
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s initialize controller failed", __func__);
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable controller failed", __func__);
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluetooth failed", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable bluetooth failed", __func__);
        return;
    }
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
   
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret){
        ESP_LOGE(BLE_TAG, "set local MTU failed, error code = %x", local_mtu_ret);
    }
    return;
}
// ============================ MAIN BLE APPLICATION ============================
void BLE_app_main(void) {
    ESP_LOGI(BLE_TAG, "Setting up BLE unit");
    // Initialize BLE stack
    Bluetooth_Initialize();
    // Create synchronization primitives
    ble_data_mutex = xSemaphoreCreateMutex();
    pid_params_mutex = xSemaphoreCreateMutex();
    BLE_recieve_queue = xQueueCreate(BLE_QUEUE_LENGTH, sizeof(ble_data_packet_t));
   
    if (BLE_recieve_queue == NULL) {
        ESP_LOGE(BLE_TAG, "Creating queue failed");
        return;
    }
    // Create data processing task
    xTaskCreatePinnedToCore(copy_ble_recieve_data_task,
                           "recieve_data_task",
                           5120,
                           NULL,
                           15,
                           &rx_task_handle,
                           0);
    ESP_LOGI(BLE_TAG, "BLE unit setup completed");
}


