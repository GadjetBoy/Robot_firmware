#include "control_1.1.h"

#define BLE_QUEUE_LENGTH 10

portMUX_TYPE ble_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ble_data_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    uint8_t                 *prepare_buf;
    int                      prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

volatile DRAM_ATTR uint8_t rx_buffer[SERVER_MAX_DATA_LEN*2];  // Stores received data
volatile DRAM_ATTR uint8_t byte_val[NUMBER_OF_BYTES];
volatile DRAM_ATTR float float_val[NUMBER_OF_FLOATS]; 


typedef struct {
    uint8_t data[SERVER_MAX_DATA_LEN];
    size_t length;
} ble_data_packet_t;

static QueueHandle_t BLE_recieve_queue;
static ble_data_packet_t current_packet;
static size_t recv_len = 0;
static SemaphoreHandle_t ble_data_mutex;
static esp_gatt_status_t status;





static const uint8_t GATTS_SERVICE_UUID_A[16] = {
    0xcb, 0x34, 0x1d, 0xed, // ed1d34cb (reversed)
    0x25, 0xf3,             // f325 (reversed)s
    0x24, 0x4f,             // 4f24 (reversed)
    0x11, 0xac,             // ac11 (reversed)
    0x7e, 0xbc, 0x1a, 0x24, 0x7a, 0x86 // Node part
};

 
static uint8_t adv_config_done = 0;
#define adv_config_flag       (1 << 0)
#define scan_rsp_config_flag  (1 << 1)


TaskHandle_t rx_task_handle;


esp_attr_value_t gatts_initial_tx_char_val =
{
    .attr_max_len = NUM_MOTORS,
    .attr_len     = 0,
    .attr_value   = NULL,
};

esp_attr_value_t gatts_initial_rx_char_val =
{
    .attr_max_len = SERVER_MAX_DATA_LEN,
    .attr_len     = 0,
    .attr_value   = (uint8_t*)rx_buffer,
};

esp_gatt_char_prop_t a_tx_property,a_rx_property;
// Add a global variable to track connection state
static bool is_connected = false;



static IRAM_ATTR void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static IRAM_ATTR void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static IRAM_ATTR bool validate_packet(const ble_data_packet_t *packet, size_t recv_len);
static IRAM_ATTR bool validate_rx_packet(const ble_data_packet_t *packet);
static void buf_reset(void);

//================ Setting GAP profile Parameters======================

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t *)GATTS_SERVICE_UUID_A,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

//=====================GAP Advertise Parameters============================

static esp_ble_adv_params_t test_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr        =
    //.peer_addr_type   =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

//============================GATTS Profile initializer Struct=======================

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    
   //charactersitics proprties for send Chractesitic
    uint16_t tx_char_handle;
    esp_bt_uuid_t tx_char_uuid;

    esp_gatt_perm_t tx_perm;
    esp_gatt_char_prop_t tx_property;

    uint16_t tx_descr_handle;
    esp_bt_uuid_t tx_descr_uuid;
    uint16_t tx_cccd_value;
    bool tx_notifications_enabled;
   
    //charactersitic for recieve
    uint16_t rx_char_handle;
    esp_bt_uuid_t rx_char_uuid;

    esp_gatt_perm_t rx_perm;
    esp_gatt_char_prop_t rx_property;
    
};

// One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT 

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       // Not get the gatt_if, so initial is ESP_GATT_IF_NONE 
    },
};

// Add these to your defines
#define PACKET_START_MARKER 0xa5
#define PACKET_END_MARKER   0x5a
#define DATA_LENGTH (NUMBER_OF_BYTES + (NUMBER_OF_FLOATS * sizeof(float)))

// Validate complete packet
bool validate_packet(const ble_data_packet_t *packet, size_t recv_len) {
    if (packet->length > CLIENT_MAX_PACKET) {
        ESP_LOGE(BLE_TAG, "Invalid packet length: %zu", packet->length);
        return true;
    }
    
    // Check start marker
    if (packet->data[0] == PACKET_START_MARKER ) {
        ESP_LOGE(BLE_TAG, "start marker recieved: 0x%02X", packet->data[0]);
        return true;
    }
    
    // Check end markers
    if (packet->data[(packet->length)-1] == PACKET_END_MARKER && recv_len != SERVER_MAX_DATA_LEN ) {
        ESP_LOGE(BLE_TAG, "end markers recieved:  0x%02X", 
                 packet->data[(packet->length)-1]);
        return true;
    }

    return false;
}

//========================GATT profile event handlers====================================

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

 switch (event){
     
     case ESP_GATTS_REG_EVT:{
         ESP_LOGI(BLE_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
         gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
         memcpy(gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid128, GATTS_SERVICE_UUID_A, 16);

         esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_A);

         esp_ble_gap_set_device_name(TEST_DEVICE_NAME);

         
         //config adv data
         esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
         if (ret){ 
            ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
         }

         adv_config_done |= adv_config_flag;
         
        break;
        }
         
     case ESP_GATTS_CREATE_EVT:{
         ESP_LOGI(BLE_TAG, "CREATE_SERVICE_EVT_TX, status %d, service_handle %" PRIu16, param->create.status, param->create.service_handle);
         gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;

         // Setup TX characteristic UUID
         gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid.len = ESP_UUID_LEN_16;
         gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TX;

         // Setup RX characteristic UUID
         ESP_LOGI(BLE_TAG, "CREATE_SERVICE_EVT_RX, status %d, service_handle %"PRIu16, param->create.status, param->create.service_handle);
         gl_profile_tab[PROFILE_A_APP_ID].rx_char_uuid.len = ESP_UUID_LEN_16;
         gl_profile_tab[PROFILE_A_APP_ID].rx_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_RX;
  
         // Start the service
         esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

         // Add RX characteristic
         a_rx_property = ESP_GATT_CHAR_PROP_BIT_WRITE;
         esp_err_t add_rx_char_ret =
         esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                 &gl_profile_tab[PROFILE_A_APP_ID].rx_char_uuid,
                                 ESP_GATT_PERM_WRITE,
                                 a_rx_property,
                                 &gatts_initial_rx_char_val,
                                 NULL); // ESP_GATT_RSP_BY_APP otherwise
         if (add_rx_char_ret){
         ESP_LOGE(BLE_TAG, "add rx_char failed, error code =%x",add_rx_char_ret);
         }

         // Add TX characteristic
         a_tx_property = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY; 
         esp_err_t add_tx_char_ret =
         esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                 &gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid,
                                 ESP_GATT_PERM_READ,
                                 a_tx_property, 
                                 &gatts_initial_tx_char_val,
                                 NULL); // ESP_GATT_RSP_BY_APP otherwise
         if (add_tx_char_ret){
         ESP_LOGE(BLE_TAG, "add tx_char failed, error code =%x",add_tx_char_ret);
         }

         
        break;
        }

     case ESP_GATTS_ADD_CHAR_EVT: {
         uint16_t length = 0; 
         const uint8_t *prf_char;

         
         esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
         if (get_attr_ret == ESP_FAIL){
               ESP_LOGE(BLE_TAG, "ILLEGAL HANDLE");
         }
         ESP_LOGI(BLE_TAG, "the gatts char length = %x", length);
         for(int i = 0; i < length; i++){
             ESP_LOGI(BLE_TAG, "prf_char[%x] = %x",i,prf_char[i]);
         }



         if(param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_TX){


            ESP_LOGI(BLE_TAG, "ADD_TX_CHAR, status %d,  attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

                 gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle = param->add_char.attr_handle;
                 gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid.len = ESP_UUID_LEN_16;
                 gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
                 
            // Add descriptor for TX characteristic 

            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                                 gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                 &gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid,
                                 ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                                 NULL,NULL);
            if (add_descr_ret){
            ESP_LOGE(BLE_TAG, "add tx_char descr failed, error code = %x", add_descr_ret);
              }
            }

         else if(param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_RX){

            ESP_LOGI(BLE_TAG, "ADD_RX_CHAR, status %d,  attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

                 gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle = param->add_char.attr_handle;
                 
            }

            
         break;
        }
    
         
     case ESP_GATTS_ADD_CHAR_DESCR_EVT: {

         if (gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid.uuid.uuid16 == param->add_char_descr.descr_uuid.uuid.uuid16) {
             gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle = param->add_char_descr.attr_handle;
             gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value = 0x000;
             ESP_LOGI(BLE_TAG, "TX CCCD Handle: %d", gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle);
            }
    
         break;
        }

     case ESP_GATTS_CONNECT_EVT: {
         esp_ble_conn_update_params_t conn_params = {0};
         memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
         //For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. 
         conn_params.latency = 0;
         conn_params.max_int = 0x30;    // max_int = 0x30*1.25ms = 40ms
         conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
         conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms

         ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, link_role %d",
             param->connect.conn_id,
             param->connect.remote_bda[0],
             param->connect.remote_bda[1],
             param->connect.remote_bda[2],
             param->connect.remote_bda[3],
             param->connect.remote_bda[4],
             param->connect.remote_bda[5],
             param->connect.link_role);

         gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
         //start sent the update connection parameters to the peer device.
         esp_ble_gap_update_conn_params(&conn_params);
         is_connected = true;
         break;
        }
    

     case ESP_GATTS_WRITE_EVT: {
         ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, handle %"PRIu16, param->write.handle);

         ble_data_packet_t recived_packet;

         if (!param->write.is_prep){ 

         recived_packet.length = param->write.len;
         size_t len = recived_packet.length;

         ESP_LOGW(BLE_TAG, "BLE data receiving length: %zu/%d",
                                 recived_packet.length, SERVER_MAX_DATA_LEN);

         if (param->write.handle == gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle) {
             //if (xSemaphoreTake(ble_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

                taskENTER_CRITICAL(&ble_data_mux);

                if (recived_packet.length + recv_len <= SERVER_MAX_DATA_LEN) {
       
                    memcpy(current_packet.data + recv_len, param->write.value, recived_packet.length);
                    recv_len += recived_packet.length;

                    if (recived_packet.length > CLIENT_MAX_PACKET) {
                     //ESP_LOGE(BLE_TAG, "Invalid packet length: %zu", recived_packet.length);
                     buf_reset();
                    }

                    memcpy(recived_packet.data, param->write.value, recived_packet.length);

                     // Check start marker
                     if (recived_packet.data[0] == PACKET_START_MARKER && recv_len !=0) {
                         //ESP_LOGW(BLE_TAG, "start marker recieved: 0x%02X", recived_packet.data[0]);
                         buf_reset();
                         memcpy(current_packet.data + recv_len, param->write.value, recived_packet.length);
                         recv_len += recived_packet.length;
                        }
    
                     // Check end markers
                     if (recived_packet.data[(recived_packet.length)-1] == PACKET_END_MARKER && recv_len != SERVER_MAX_DATA_LEN ) {
                         //ESP_LOGE(BLE_TAG, "end markers recieved:  0x%02X", 
                         //recived_packet.data[(recived_packet.length)-1]);
                         buf_reset();
                        }


                    if (recv_len >= SERVER_MAX_DATA_LEN) {
                        //recv_len = (recv_len > SERVER_MAX_DATA_LEN) ? SERVER_MAX_DATA_LEN : recv_len;
                        current_packet.length = recv_len;

                        ble_data_packet_t queue_packet;
                        memcpy(&queue_packet, &current_packet, sizeof(queue_packet));
                        

                        if (xQueueSend(BLE_recieve_queue, &queue_packet, pdMS_TO_TICKS(10)) != pdPASS) {
                            //ESP_LOGE(BLE_TAG, "Queue full - packet dropped");
                        } else {
                            //ESP_LOGI(BLE_TAG, "BLE data received complete");
                            //ESP_LOGW(BLE_TAG, "Total BLE data received: %zu/%d",
                                // recv_len, SERVER_MAX_DATA_LEN);
                            //ESP_LOGE(BLE_TAG, "Byte[0] = %u (0x%02X)",
                                     //current_packet.data[1], current_packet.data[1]);
                        }

                        recv_len = 0;
                        recived_packet.length = 0;
                        memset(&current_packet, 0, sizeof(current_packet));
                    } else {
                       // ESP_LOGW(BLE_TAG, "BLE data receiving in progress: %zu/%d",
                                // recv_len, SERVER_MAX_DATA_LEN);
                    }
                } else {
                   // ESP_LOGE(BLE_TAG, "Buffer overflow or Data corruption: recv_len=%zu + len=%zu > %d",
                            // recv_len,  current_packet.length, SERVER_MAX_DATA_LEN);
                    recv_len = 0;
                    memset(&current_packet, 0, sizeof(current_packet));
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
                  //xSemaphoreGive(ble_data_mutex);
                  taskEXIT_CRITICAL(&ble_data_mux);

             // }
            }

            // Check if the write is for the TX CCCD
          else if (param->write.handle == gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle && len==2) {
             uint16_t cccd_value = param->write.value[1]<<8 | param->write.value[0];
             gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value = cccd_value;
             ESP_LOGI(BLE_TAG, "CCCD Write: %04x", cccd_value);

                 if (cccd_value == 0x0001){
                     if (a_tx_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                          ESP_LOGI(BLE_TAG, "notify enable");

                          gl_profile_tab[PROFILE_A_APP_ID].tx_notifications_enabled = true;
                      
                        }
                
                    }
                  else {
                     ESP_LOGI(BLE_TAG, "Notifications disabled");
                     gl_profile_tab[PROFILE_A_APP_ID].tx_notifications_enabled = false;
                    }
                
                }
                
            }

         write_event_env(gatts_if, &prepare_write_env, param);

         if (param->write.need_rsp){

             esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
              if (gatt_rsp) {
                  gatt_rsp->attr_value.len = param->write.len;
                  gatt_rsp->attr_value.handle = param->write.handle;
                  gatt_rsp->attr_value.offset = param->write.offset;
                  gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                  memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);

                 esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                 if (response_err != ESP_OK) {
                     ESP_LOGE(BLE_TAG, "Send response error\n");
                    }
                 free(gatt_rsp);
                } 
             else {
                 ESP_LOGE(BLE_TAG, "malloc failed, no resource to send response error\n");
                 status = ESP_GATT_NO_RESOURCES;
                }
             if (status != ESP_GATT_OK){
                 return;
                }
            }
     
          break;
        }

     case ESP_GATTS_EXEC_WRITE_EVT: {
         if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
             // Copy into safe packet
             ble_data_packet_t packet;
             memcpy(packet.data, prepare_write_env.prepare_buf, prepare_write_env.prepare_len);
             packet.length = prepare_write_env.prepare_len;

             // Push to queue
             xQueueSend(BLE_recieve_queue, &packet, portMAX_DELAY);

             ESP_LOGI(BLE_TAG, "BLE data received complete, length=%d", packet.length);
            }
         // Cleanup
         if (prepare_write_env.prepare_buf) {
             free(prepare_write_env.prepare_buf);
             prepare_write_env.prepare_buf = NULL;
             prepare_write_env.prepare_len = 0;
            }
          break;
        }

     case ESP_GATTS_DISCONNECT_EVT:{

         ESP_LOGI(BLE_TAG, "ClIENT DISCONNECTED, conn_id %d, reason %d", param-> disconnect.conn_id,param-> disconnect.reason);
         gl_profile_tab[PROFILE_A_APP_ID].conn_id = 0;

         ESP_LOGI(BLE_TAG, "CLIENT DISCONNECTED, resetting buffers");
    
         if (xSemaphoreTake(ble_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
           recv_len = 0;
           memset(&current_packet, 0, sizeof(current_packet));
           xSemaphoreGive(ble_data_mutex);
          }
    
          // Clear queue
          xQueueReset(BLE_recieve_queue);

         esp_ble_gap_start_advertising(&test_adv_params);

         is_connected = false;
        
         break;
        }

     case ESP_GATTS_RESPONSE_EVT: {
         ESP_LOGD(BLE_TAG, "GATT_RESPONSE_EVT: status %d, trans_id %d", param->rsp.status, param->rsp.conn_id);  // Optional: Debug only
         // No action needed—confirms response sent
         break;
        }

      default:{
         ESP_LOGI(BLE_TAG, "Unhandled GATTS event: %d", event);
         break;
        }


    }
    
}

static IRAM_ATTR void buf_reset(void){
  //ESP_LOGE(BLE_TAG,"recieve packet ERROR:recieve buffer reset");
  //ESP_LOG_BUFFER_HEX(BLE_TAG, current_packet.data,sizeof(current_packet.data));
  memset(&current_packet, 0, sizeof(current_packet));
  recv_len=0;
}
    



//======================================GAP Event Handler=====================================

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~adv_config_flag);
    if (param->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(BLE_TAG, "Advertising start failed");
    } else if (adv_config_done == 0) {
        esp_ble_gap_start_advertising(&test_adv_params);
    }
    break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:{
     ESP_LOGI(BLE_TAG, "update connection params status = %d, conn_int = %d,latency = %d, timeout = %d",
              param->update_conn_params.status,
              param->update_conn_params.conn_int,
              param->update_conn_params.latency,
              param->update_conn_params.timeout);
         break;
    }

    default:
    ESP_LOGI(BLE_TAG, "Unhandled event: %d", event);
    break;
    }

}


//===================================GATT event handlers===================================

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // If event is register event, store the gatts_if for each profile 
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(BLE_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

 // If the gatts_if equal to profile A, call profile A cb handler,
 //so here call each profile's callback 
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE||gatts_if == gl_profile_tab[idx].gatts_if)             {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);


}

void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
 status = ESP_GATT_OK;
    //if (param->write.need_rsp){
        if (param->write.is_prep) {
            if(param->write.offset > SERVER_MAX_DATA_LEN) {
                status = ESP_GATT_INVALID_OFFSET;
            } else if ((param->write.offset + param->write.len) > SERVER_MAX_DATA_LEN) {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }

            if (status == ESP_GATT_OK){

             if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
                 prepare_write_env->prepare_buf = (uint8_t *)malloc(SERVER_MAX_DATA_LEN*sizeof(uint8_t));
                 prepare_write_env->prepare_len = 0;
                 if (prepare_write_env->prepare_buf == NULL) {
                     ESP_LOGE(BLE_TAG, "Gatt_server prep no mem");
                     status = ESP_GATT_NO_RESOURCES;
                    }
                }
             if (xSemaphoreTake(ble_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE){
                  memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);

                  prepare_write_env->prepare_len += param->write.len;
                }

             xSemaphoreGive(ble_data_mutex);

            }
        } 
    //}
}

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){

     if (xSemaphoreTake(ble_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE){
         if (prepare_write_env->prepare_len > 0){
             if( xQueueSend( BLE_recieve_queue,( void * ) prepare_write_env->prepare_buf,( TickType_t ) 10 ) != pdPASS ){
                          
                 ESP_LOGE(BLE_TAG, "The Queue is Full Failed to Passs Data" );

                 return;
                }
            }
        }
     xSemaphoreGive(ble_data_mutex);
     //ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else{
        ESP_LOGI(BLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}



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
        ESP_LOGE(BLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    return;
}

bool validate_rx_packet(const ble_data_packet_t *packet) {
    if (packet->length != SERVER_MAX_DATA_LEN) {
        ESP_LOGE(BLE_TAG, "Invalid packet length: %zu", packet->length);
        return false;
    }
    
    // Check start marker
    if (packet->data[0] != PACKET_START_MARKER) {
        ESP_LOGE(BLE_TAG, "start marker recieved: 0x%02X", packet->data[0]);
        return false;
    }
    
    // Check end markerss
    if (packet->data[(packet->length)-1] != PACKET_END_MARKER) {
        ESP_LOGE(BLE_TAG, "end markers recieved:  0x%02X", 
                 packet->data[(packet->length)-1]);
        return false;
    }

    return true;
}


void copy_ble_recieve_data_task(void *pvParameters) {
    uint8_t log_counter = 0;
    int offset = 0;
    static ble_data_packet_t local_rx_buffer;

    while (1) {

         offset=0;

         if (xQueueReceive(BLE_recieve_queue, &local_rx_buffer, portMAX_DELAY) == pdTRUE){
             
             bool validate = validate_rx_packet(&local_rx_buffer);

             if(!validate){
                 ESP_LOGE(BLE_TAG, "Queue contained invalid packet - skipping");
                 ESP_LOG_BUFFER_HEX(BLE_TAG, local_rx_buffer.data,sizeof(local_rx_buffer.data));
                 continue;  // Skip to next packet 
                }

              // Skip first parity byte
              offset += 1;

             taskENTER_CRITICAL(&ble_mux);
             // Copy ints
             memcpy((void*)byte_val, (const void*)(local_rx_buffer.data + offset), NUMBER_OF_BYTES);
             offset += NUMBER_OF_BYTES;

             // Copy floats
             memcpy((void*)float_val, (const void*)(local_rx_buffer.data + offset), NUMBER_OF_FLOATS * sizeof(float));
             offset += NUMBER_OF_FLOATS * sizeof(float);

             taskEXIT_CRITICAL(&ble_mux);

             offset+=2;

             // Log floats (every 5th packet)
             log_counter++;
             if (log_counter == 10) {
                // Log integers
                for (int i = 0; i < NUMBER_OF_BYTES; i++) {
                   ESP_LOGW(BLE_TAG, "Byte[%d] = %u (0x%02X)", i, byte_val[i], byte_val[i]);
                }
                 for (int i = 0; i < NUMBER_OF_FLOATS; i++) {
                     ESP_LOGW(BLE_TAG, "Float[%d] = %.2f", i, float_val[i]);
                    }
                  //ESP_LOG_BUFFER_HEX(BLE_TAG, local_rx_buffer.data,sizeof(local_rx_buffer.data));
                 log_counter = 0;
                }


            }
           
            // Reset buffer for next packe
         offset=0;
         memset(&local_rx_buffer, 0, sizeof(uint8_t)*SERVER_MAX_DATA_LEN);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void BLE_app_main(void) {
  ESP_LOGI(BLE_TAG, "setting up BLE unit\n" );
  memset((void*)float_val, 0, sizeof(float_val)); 
  memset((void*)byte_val, 0, sizeof(byte_val)); 

  Bluetooth_Initialize();

  ble_data_mutex = xSemaphoreCreateMutex();

  BLE_recieve_queue = xQueueCreate(BLE_QUEUE_LENGTH, sizeof(ble_data_packet_t));
  if (BLE_recieve_queue == NULL) {
        ESP_LOGE(BLE_TAG, "Creating queue failed");     
        return;
    }

    
    xTaskCreatePinnedToCore(copy_ble_recieve_data_task, 
                "recieve_data_task", 
                 5120, 
                 NULL, 
                 15,
                 &rx_task_handle, 
                 1);


   ESP_LOGI(BLE_TAG, "BLE unit setiing up completed\n");
    
}

