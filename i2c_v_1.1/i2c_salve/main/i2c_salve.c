#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_slave.h"
#include <string.h>

#define I2C_PORT                 I2C_NUM_0  
#define I2C_SLAVE_SCL_IO         13
#define I2C_SLAVE_SDA_IO         15
#define SLAVE_ADDR               0x77

#define TAG "SLAVE"

#define DATA_LENGTH              18
#define QUEUE_LENGTH             5

i2c_slave_dev_handle_t slave_handle;
i2c_slave_rx_done_event_data_t rx_data;
//static QueueHandle_t en_data_receive_queue = NULL;
uint8_t *data_rd ;


void I2C_Salve_Device_Setup(i2c_slave_dev_handle_t *slave_handle);
uint8_t Data_Recive(QueueHandle_t *en_data_receive_queue,uint8_t a);
void Data_Recive_Setup(QueueHandle_t *en_data_receive_queue);
static bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data);


void app_main(void)
{
 
 QueueHandle_t en_data_receive_queue = xQueueCreate(QUEUE_LENGTH, sizeof(i2c_slave_rx_done_event_data_t));
 
 // setup the slave
 I2C_Salve_Device_Setup(&slave_handle);

 ESP_LOGI(TAG, "Slave initialized and ready to receive data");
 
 Data_Recive_Setup(& en_data_receive_queue);
 
 uint8_t a = 0;
 
 
 while(1){
     a = Data_Recive(&en_data_receive_queue,a);
    }

 
}

/***** I2C Slave Device Setup *****/

void I2C_Salve_Device_Setup(i2c_slave_dev_handle_t *slave_handle){
   
 //configure slave 
 i2c_slave_config_t i2c_slv_config = {
     .addr_bit_len = I2C_ADDR_BIT_LEN_7,
     .clk_source = I2C_CLK_SRC_DEFAULT,
     .i2c_port = I2C_PORT,
     .send_buf_depth = 256,
     .scl_io_num = I2C_SLAVE_SCL_IO,
     .sda_io_num = I2C_SLAVE_SDA_IO,
     .slave_addr =SLAVE_ADDR,
     .intr_priority = 2,
    };

 //i2c_slave_dev_handle_t slave_handle;
 ESP_ERROR_CHECK(i2c_new_slave_device(& i2c_slv_config, slave_handle));

}

/***** Data Reciever *****/
void Data_Recive_Setup(QueueHandle_t *en_data_receive_queue){

 i2c_slave_event_callbacks_t Call_back = {
     .on_recv_done =  i2c_slave_rx_done_callback,
    };

 //Registering CallBack Function
 ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_handle, &Call_back,*en_data_receive_queue));

}

/*** Recieve Data from Master***/

uint8_t Data_Recive(QueueHandle_t *en_data_receive_queue,uint8_t a){

 // Clear any leftover events from previous transactions.

 xQueueReset(*en_data_receive_queue);

 ESP_LOGE(TAG, "Receiving ..... \n");

 if(a==0){
     //allocte memeory /recive buffer 
     data_rd = (uint8_t*) calloc(DATA_LENGTH, sizeof(uint8_t));

     ESP_ERROR_CHECK(i2c_slave_receive(slave_handle, data_rd,DATA_LENGTH));
    }

 if(xQueueReceive(*en_data_receive_queue, &rx_data, pdMS_TO_TICKS(10000))==pdTRUE){
     
      // Re arrange the 8bit integer values to form 16bit values

      union {
         uint8_t received_data[DATA_LENGTH] ;
      	 int16_t en_values[(DATA_LENGTH/2)];
        }organizer;

        memcpy(organizer.received_data, data_rd, DATA_LENGTH);

        ESP_LOGE(TAG, "Data Received \n");

     for (int j = 0 ; j < (DATA_LENGTH/2); j++){
         
         ESP_LOGI(TAG,  "Encorder[%d] Value : %d \n", (j+1),(int)organizer.en_values[j]);

        }

         ESP_LOGE(TAG,".............\n");

         free (data_rd);

         return 0 ;

    }
    
 else {
        
     ESP_LOGE(TAG, "Receive timeout dude\n");

        return 1 ;

    }

    
    
}

/***** call back function *****/

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data){

    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;

}