#pragma once

#include "control_1.3.h"

#ifndef BLE_UNIT_H
#define BLE_UNIT_H

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"



#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * BLE CONFIGURATION CONSTANTS
 * --------------------------------------------------------------------------- */
// Global definitions and constants
#define BLE_TAG "BLE"
#define PROFILE_A_APP_ID 0
#define PROFILE_NUM 1
#define GATTS_NUM_HANDLE_A 7
#define DEVICE_NAME "ESP32_BLE_Server"

// UUID Definitions
#define GATTS_CHAR_UUID_TX 0xFF01
#define GATTS_CHAR_UUID_RX 0xFF02

// Packet configuration
#define PACKET_START_MARKER 0xA5
#define PACKET_END_MARKER 0x5A

#define SERVER_MAX_DATA_LEN 68
#define CLIENT_MAX_PACKET 20
#define BLE_QUEUE_LENGTH 10

#define NUMBER_OF_BYTES 1
#define NUMBER_OF_FLOATS 16

/* ---------------------------------------------------------------------------
 * BLE DATA PACKET STRUCTURE
 * --------------------------------------------------------------------------- */
typedef struct {
    uint8_t data[SERVER_MAX_DATA_LEN];
    uint16_t length;
} ble_data_packet_t;

/* ---------------------------------------------------------------------------
 * PREPARE WRITE ENVIRONMENT
 * --------------------------------------------------------------------------- */
typedef struct {
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

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

/* ---------------------------------------------------------------------------
 * GLOBALS (extern)
 * --------------------------------------------------------------------------- */
extern portMUX_TYPE ble_mux;

volatile extern uint8_t byte_val[NUMBER_OF_BYTES];
volatile extern float float_val[NUMBER_OF_FLOATS];


/* ---------------------------------------------------------------------------
 * FUNCTION PROTOTYPES
 * --------------------------------------------------------------------------- */

// ============================ FORWARD DECLARATIONS ============================

void BLE_app_main(void);


#ifdef __cplusplus
}
#endif

#endif /* BLE_UNIT_H */



