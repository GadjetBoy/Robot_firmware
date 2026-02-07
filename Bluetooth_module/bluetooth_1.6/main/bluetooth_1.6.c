#include <stdio.h>
#include <inttypes.h>

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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

#define GATTS_TAG "BTserver"
#define TAG0      "BTwriteEvent"
#define PROFILE_A_APP_ID 0
#define PROFILE_NUM 1
#define TEST_DEVICE_NAME "ESP32_server"  
#define PREPARE_BUF_MAX_SIZE 32

portMUX_TYPE ble_mux = portMUX_INITIALIZER_UNLOCKED;


#define GATT_PROFILE_SERVICE_INST 0

static const uint8_t GATTS_SERVICE_UUID_A[16] = {
    0xcb, 0x34, 0x1d, 0xed, // ed1d34cb (reversed)
    0x25, 0xf3,             // f325 (reversed)
    0x24, 0x4f,             // 4f24 (reversed)
    0x11, 0xac,             // ac11 (reversed)
    0x7e, 0xbc, 0x1a, 0x24, 0x7a, 0x86 // Node part
};

#define GATTS_CHAR_UUID_TX 0xFF01  
#define GATTS_CHAR_UUID_RX 0xFF02
#define GATTS_NUM_HANDLE_A 7
 

#define NUMBER_OF_MOTORS 8 
#define SERVER_MAX_DATA_LEN 42
#define CLIENT_MAX_PACKET 20
#define NUMBER_OF_INTS 7
#define NUMBER_OF_FLOATS 8  

static uint8_t adv_config_done = 0;
#define adv_config_flag       (1 << 0)
#define scan_rsp_config_flag  (1 << 1)

// Buffers for sensor data
static uint8_t tx_sensor_data[NUMBER_OF_MOTORS] = {0,1,2,3,4,5,6,7};  // Holds data to send

uint8_t rx_buffer[SERVER_MAX_DATA_LEN];  // Stores received data
uint8_t int_val[NUMBER_OF_INTS];
float float_val[SERVER_MAX_DATA_LEN-(NUMBER_OF_INTS+3)]={0.0};
static size_t  recv_len = 0;

esp_attr_value_t gatts_initial_tx_char_val =
{
    .attr_max_len = NUMBER_OF_MOTORS,
    .attr_len     = NUMBER_OF_MOTORS,
    .attr_value   = tx_sensor_data,
};

esp_attr_value_t gatts_initial_rx_char_val =
{
    .attr_max_len = NUMBER_OF_MOTORS,
    .attr_len     = NUMBER_OF_MOTORS,
    .attr_value   = rx_buffer,
};

esp_gatt_char_prop_t a_tx_property,a_rx_property;
// Add a global variable to track connection state
static bool is_connected = false;



static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
 TaskHandle_t tx_task_handle;

/********* Setting GAP profile Parameters********/

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

/***********GAP Advertise Parameters***************/

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

/***********GATTS Profile initializer Struct**************/

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

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       // Not get the gatt_if, so initial is ESP_GATT_IF_NONE 
    },
};


/****************GATT profile event handlers*************/

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
 switch (event) {
    
    case ESP_GATTS_REG_EVT:{
         ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
         gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
         memcpy(gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid128, GATTS_SERVICE_UUID_A, 16);

         esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_A);

         esp_ble_gap_set_device_name(TEST_DEVICE_NAME);

         
         //config adv data
         esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
         if (ret){ 
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
         }

         adv_config_done |= adv_config_flag;
         
        break;
        }
         
    case ESP_GATTS_CREATE_EVT:{
         ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT_TX, status %d, service_handle %" PRIu16, param->create.status, param->create.service_handle);
         gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;

         // Setup TX characteristic UUID
         gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid.len = ESP_UUID_LEN_16;
         gl_profile_tab[PROFILE_A_APP_ID].tx_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TX;

         // Setup RX characteristic UUID
         ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT_RX, status %d, service_handle %"PRIu16, param->create.status, param->create.service_handle);
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
         ESP_LOGE(GATTS_TAG, "add rx_char failed, error code =%x",add_rx_char_ret);
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
         ESP_LOGE(GATTS_TAG, "add tx_char failed, error code =%x",add_tx_char_ret);
         }

         
        break;
        }

    case ESP_GATTS_ADD_CHAR_EVT: {
         uint16_t length = 0; 
         const uint8_t *prf_char;

         
         esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
         if (get_attr_ret == ESP_FAIL){
               ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
         }
         ESP_LOGI(GATTS_TAG, "the gatts char length = %x", length);
         for(int i = 0; i < length; i++){
             ESP_LOGI(GATTS_TAG, "prf_char[%x] = %x",i,prf_char[i]);
         }



         if(param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_TX){


            ESP_LOGI(GATTS_TAG, "ADD_TX_CHAR, status %d,  attr_handle %d, service_handle %d",
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
            ESP_LOGE(GATTS_TAG, "add tx_char descr failed, error code = %x", add_descr_ret);
              }
            }

         else if(param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_RX){

            ESP_LOGI(GATTS_TAG, "ADD_RX_CHAR, status %d,  attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

                 gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle = param->add_char.attr_handle;
                 
            }

            
         break;
        }
    
         
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {

      if (gl_profile_tab[PROFILE_A_APP_ID].tx_descr_uuid.uuid.uuid16 == param->add_char_descr.descr_uuid.uuid.uuid16) {
           gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle = param->add_char_descr.attr_handle;
           gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value = 0x000;
           ESP_LOGI(GATTS_TAG, "TX CCCD Handle: %d", gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle);
        }
    
     break;
    }

    case ESP_GATTS_CONNECT_EVT: {
     esp_ble_conn_update_params_t conn_params = {0};
     memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

     /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
     conn_params.latency = 0;
     conn_params.max_int = 0x30;    // max_int = 0x30*1.25ms = 40ms
     conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
     conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms

     ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, link_role %d",
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
    

    case ESP_GATTS_READ_EVT: {
      ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16,
              param->read.conn_id, param->read.trans_id, param->read.handle);
             
              esp_gatt_rsp_t read_rsp;
              memset(&read_rsp, 0, sizeof(esp_gatt_rsp_t));
              read_rsp.attr_value.handle = param->read.handle;
              
              if (param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle) {
              read_rsp.attr_value.len = NUMBER_OF_MOTORS;
              read_rsp.attr_value.value[0] = tx_sensor_data[0];
              read_rsp.attr_value.value[1] = tx_sensor_data[1];
              read_rsp.attr_value.value[2] = tx_sensor_data[2];
              read_rsp.attr_value.value[3] = tx_sensor_data[3];
              read_rsp.attr_value.value[4] = tx_sensor_data[4];
              read_rsp.attr_value.value[5] = tx_sensor_data[5];
              read_rsp.attr_value.value[6] = tx_sensor_data[6];
              read_rsp.attr_value.value[7] = tx_sensor_data[7];
              
              // Send response back to client
              esp_ble_gatts_send_response(gatts_if,
                                          param->read.conn_id,
                                          param->read.trans_id,
                                          ESP_GATT_OK, &read_rsp);
             }

             else if(param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle&&read_rsp.attr_value.len == 2){
                 //read_rsp.attr_value.len = 2 // CCCD is 16 bits (2 bytes)
                 read_rsp.attr_value.value[0] = gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value & 0xFF; // Low byte
                 read_rsp.attr_value.value[1] = (gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value >> 8) & 0xFF; // High byte

                 esp_ble_gatts_send_response(gatts_if,
                                param->read.conn_id,
                                param->read.trans_id,
                                ESP_GATT_OK, &read_rsp);
}

             else{
                 ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, INVALID REQUEST FROM :conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16,
                 param->read.conn_id, param->read.trans_id, param->read.handle);
             
                 // Send response back to client
                 esp_ble_gatts_send_response(gatts_if,
                                          param->read.conn_id,
                                          param->read.trans_id,
                                          ESP_GATT_OK, &read_rsp);
            }
             
     break;
    }

    case ESP_GATTS_WRITE_EVT: {
       ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle %"PRIu16, param->write.handle);
       
      if (!param->write.is_prep){

         size_t len = param->write.len;

         if (param->write.handle == gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle) {
              // Store received data into buffer
            
              if(len+recv_len<=SERVER_MAX_DATA_LEN){
                 memcpy(rx_buffer+recv_len, param->write.value, len);
                 recv_len+=len;

                }
              else {
                 // overflow — probably a bug or garbled client write
                 ESP_LOGW(GATTS_TAG, "Too much data: recv_len=%d + len=%d > %d",
                 recv_len, len, SERVER_MAX_DATA_LEN);
                 recv_len = 0;
                 break;
                }

              if(recv_len == SERVER_MAX_DATA_LEN){

                /* uint8_t start_meta = rx_buffer[0];
                 uint8_t end_meta[2];
                 end_meta[0]=rx_buffer[SERVER_MAX_DATA_LEN-2];
                 end_meta[1]=rx_buffer[SERVER_MAX_DATA_LEN-1];*/

                 // copydown integer values 
                   taskENTER_CRITICAL(&ble_mux);
                   memcpy(int_val, rx_buffer + 1, NUMBER_OF_INTS);
                   
                 //copydown float values 
                   
                   memcpy(float_val,rx_buffer+NUMBER_OF_INTS+1,NUMBER_OF_FLOATS*sizeof(float));
                   taskEXIT_CRITICAL(&ble_mux);
                // Log integers
                 for (int i = 0; i < 7; i++) {
                     ESP_LOGI(TAG0, "Int[%d] = %u", i, int_val[i]);
                    }

                // Log floats
                 for (int i = 0; i < 8; i++) {
                     ESP_LOGI(TAG0, "Float[%d] = %.2f", i, float_val[i]);
                     //printf("Float value: %.2f\n", float_val[i]);
                    }

                // Reset buffer for next packet
                  recv_len = 0;

                }

              
             
            }

            // Check if the write is for the TX CCCD
          else if (param->write.handle == gl_profile_tab[PROFILE_A_APP_ID].tx_descr_handle && len==2) {
             uint16_t cccd_value = param->write.value[1]<<8 | param->write.value[0];
             gl_profile_tab[PROFILE_A_APP_ID].tx_cccd_value = cccd_value;
             ESP_LOGI(GATTS_TAG, "CCCD Write: %04x", cccd_value);

                 if (cccd_value == 0x0001){
                     if (a_tx_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                          ESP_LOGI(GATTS_TAG, "notify enable");

                          gl_profile_tab[PROFILE_A_APP_ID].tx_notifications_enabled = true;
                      
                        }
                
                    }
                  else {
                     ESP_LOGI(GATTS_TAG, "Notifications disabled");
                     gl_profile_tab[PROFILE_A_APP_ID].tx_notifications_enabled = false;
                    }
                
                }  
                
        }

      else{

            ESP_LOGE(GATTS_TAG, "invlaid write to the server\n");
        }
     esp_gatt_status_t status = ESP_GATT_OK;
     esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp) {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);

                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK) {
                    ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            } 
            else {
                ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK){
                return;
            }
     break;
    }

    case ESP_GATTS_DISCONNECT_EVT:{

         ESP_LOGI(GATTS_TAG, "ClIENT DISCONNECTED, conn_id %d, reason %d", param-> disconnect.conn_id,param-> disconnect.reason);
         gl_profile_tab[PROFILE_A_APP_ID].conn_id = 0;

         esp_ble_gap_start_advertising(&test_adv_params);

         is_connected = false;
        
        break;
        }

    default:{
         ESP_LOGI(GATTS_TAG, "Unhandled GATTS event: %d", event);
         break;
        }
}

}

/************GAP Event Handler*****************/

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~adv_config_flag);
    if (param->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(GATTS_TAG, "Advertising start failed");
    } else if (adv_config_done == 0) {
        esp_ble_gap_start_advertising(&test_adv_params);
    }
    break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:{
     ESP_LOGI(GATTS_TAG, "update connection params status = %d, conn_int = %d,latency = %d, timeout = %d",
              param->update_conn_params.status,
              param->update_conn_params.conn_int,
              param->update_conn_params.latency,
              param->update_conn_params.timeout);
         break;
    }

    default:
    ESP_LOGI(GATTS_TAG, "Unhandled event: %d", event);
    break;
    }

}


/****************GATT event handlers*************/

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

 /* If the gatts_if equal to profile A, call profile A cb handler,
 * so here call each profile's callback */
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



void Bluetooth_Initialize()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    return;
}

void ble_send_sensor_data_task(void *pvParameters) {
  while(1) {
    if (gl_profile_tab[PROFILE_A_APP_ID].tx_notifications_enabled && is_connected) {
      // Update tx_sensor_data with new sensor values here

      esp_ble_gatts_send_indicate(
        gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
        gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
        sizeof(tx_sensor_data),
        tx_sensor_data,
        false // false = Notification (no ack), true = Indication (ack)
      );
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust interval as needed
  }
}

void app_main(void) {
    Bluetooth_Initialize();

    xTaskCreatePinnedToCore(ble_send_sensor_data_task, 
                "BLE_send_sensor_data_task", 
                 4096, 
                 NULL, 
                 5,
                 &tx_task_handle, 
                 0);
}

