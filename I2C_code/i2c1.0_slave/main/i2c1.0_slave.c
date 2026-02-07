#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_slave.h"
#include <string.h>

#define I2C_PORT                 I2C_NUM_0  
#define I2C_SLAVE_SCL_IO         18
#define I2C_SLAVE_SDA_IO         17
#define SLAVE_ADDR               0x78

#define TAG "SLAVE"
static QueueHandle_t s_receive_queue = NULL;

#define MAX_DATA_LEN 18
#define QUEUE_LENGTH 5

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void app_main(void)
{
 
 // configure slave 
 i2c_slave_config_t i2c_slv_config = {
     .addr_bit_len = I2C_ADDR_BIT_LEN_7,
     .clk_source = I2C_CLK_SRC_DEFAULT,
     .i2c_port = I2C_PORT,
     .send_buf_depth = 256,
     .scl_io_num = I2C_SLAVE_SCL_IO,
     .sda_io_num = I2C_SLAVE_SDA_IO,
     .slave_addr =SLAVE_ADDR,
      };

 i2c_slave_dev_handle_t slave_handle;
 ESP_ERROR_CHECK(i2c_new_slave_device(& i2c_slv_config, &slave_handle));

 ESP_LOGI(TAG, "Slave initialized and ready to receive data");

 //i2c slave recieve buffer initialize

 //void *in_buffer = malloc(MAX_DATA_LEN);
 //uint8_t *data_rd = (uint8_t *)in_buffer ;

s_receive_queue = xQueueCreate(QUEUE_LENGTH, sizeof(i2c_slave_rx_done_event_data_t));
i2c_slave_event_callbacks_t cbs = {
    .on_recv_done = i2c_slave_rx_done_callback,
};
ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_handle, &cbs, s_receive_queue));

i2c_slave_rx_done_event_data_t rx_data;

//ESP_ERROR_CHECK(i2c_slave_receive(slave_handle, data_rd, MAX_DATA_LEN));

while(1){

 //void *in_buffer = malloc(MAX_DATA_LEN);
 //uint8_t *data_rd = (uint8_t *)in_buffer ;
 uint8_t *data_rd = (uint8_t *) calloc(MAX_DATA_LEN, sizeof(uint8_t));

 ESP_ERROR_CHECK(i2c_slave_receive(slave_handle, data_rd, MAX_DATA_LEN));

if(xQueueReceive(s_receive_queue, &rx_data, pdMS_TO_TICKS(10000))==pdTRUE){

     // Ensure null termination for string printing
            
        char received_str[MAX_DATA_LEN] = {0};
      //char *received_str ;
     //received_str = (char*)data_rd;

    memcpy(received_str,(char*) data_rd, MAX_DATA_LEN-1);
    // Append the null terminator.
    received_str[MAX_DATA_LEN] = '\0';


      ESP_LOGI(TAG, "Data: %s \n", received_str );
    }
    
else {
        
        ESP_LOGE(TAG, "Receive timeout dude\n");
    }
  
   // free(in_buffer);
    free(data_rd);
  //vTaskDelay(1000/portTICK_PERIOD_MS);
}
  

 }
 

 /*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_slave.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "I2C_SLAVE_EXAMPLE";

#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA_GPIO
#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL_GPIO
#define I2C_SLAVE_ADDRESS CONFIG_I2C_SLAVE_ADDRESS
#define DATA_LENGTH 16

static QueueHandle_t receive_queue;
static uint8_t data_buffer;

static bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(queue, data_buffer, &high_task_wakeup); // Send the received buffer to a task
    return (high_task_wakeup == pdTRUE);
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C slave");

    i2c_slave_config_t config = {
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .slave_addr = I2C_SLAVE_ADDRESS,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .max_transfer_sz = DATA_LENGTH, // Set maximum transfer size
    };

    i2c_slave_dev_handle_t slave_handle;
    ESP_ERROR_CHECK(i2c_slave_driver_install(I2C_NUM_0, &config, 0, &slave_handle));

    receive_queue = xQueueCreate(10, sizeof(data_buffer));
    if (receive_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create receive queue");
        return;
    }

    i2c_slave_event_callbacks_t cbs = {
        .on_recv_done = i2c_slave_rx_done_callback,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_handle, &cbs, receive_queue));

    while (1) {
        ESP_LOGI(TAG, "Waiting for data from master...");
        ESP_ERROR_CHECK(i2c_slave_receive(slave_handle, data_buffer, DATA_LENGTH, portMAX_DELAY));
        uint8_t received_data;
        if (xQueueReceive(receive_queue, received_data, pdMS_TO_TICKS(10000))) {
            ESP_LOGI(TAG, "Data received:");
            for (int i = 0; i < DATA_LENGTH; i++) {
                ESP_LOGI(TAG, "Data[%d]: 0x%02X", i, received_data[i]);
            }
        } else {
            ESP_LOGW(TAG, "Receive timeout");
        }
    }

    ESP_ERROR_CHECK(i2c_slave_driver_delete(I2C_NUM_0));
} */