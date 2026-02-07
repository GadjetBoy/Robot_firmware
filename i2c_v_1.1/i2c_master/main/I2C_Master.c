#include "Decorder.h"
static const char *TAG = "I2C MASTER";

#define I2C_MASTER_SCL_IO          13
#define I2C_MASTER_SDA_IO          15
#define I2C_PORT_NUM               I2C_NUM_0   // Using I2C port 0
#define I2C_MASTER_FREQ_HZ         200000      // I2C clock frequency (100 kHz)
#define I2C_MASTER_GLITCH_FILTER   7           // Glitch filter threshold

#define SLAVE_ADDR                 0x77        // 7-bit I2C address of the slave device
#define DATA_LENGTH                8
#define LED_PIN                    12

i2c_master_dev_handle_t device_handle;

void I2C_Master_setup();
void Send_data_Task( void* Parameter);
void Blink_Task(void* Parameter);
void PCNT_Print_Task(void *Parameters);


void app_main(void){

 //set PCNT
 PCNT_app_main();
 
 //set I2c master 
 ESP_LOGI(TAG, "setting up I2C MASTER device" );
 I2C_Master_setup() ;
 ESP_LOGI(TAG, "setting up I2C MASTER device Complete");
 //send data to slave

 ESP_LOGI(TAG, "setting up PCNT Count Print_Tasks" );

 
 ESP_LOGI(TAG, "setting up I2C Send_Tasks" );

 TaskHandle_t send_Handle = NULL;
 xTaskCreatePinnedToCore(Send_data_Task, 
                         "Send_task", 
                          4096,
                          NULL,                //(void*)Data_buff, 
                          20,
                          &send_Handle,
                          1);

gpio_reset_pin(LED_PIN);
gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); 
gpio_set_level(LED_PIN, 1);
vTaskDelay(pdMS_TO_TICKS(500));
gpio_set_level(LED_PIN, 0);

ESP_LOGI(TAG, "setting up I2C MASTER device finish" );
 
}

//======================= Master Setup =============================/

void I2C_Master_setup(void){

 //i2c masterBus Config

 i2c_master_bus_config_t master_config={
     
     .clk_source = I2C_CLK_SRC_DEFAULT,
     .i2c_port = I2C_PORT_NUM,
     .scl_io_num = I2C_MASTER_SCL_IO,
     .sda_io_num = I2C_MASTER_SDA_IO,
     .glitch_ignore_cnt = I2C_MASTER_GLITCH_FILTER,
     .flags.enable_internal_pullup = false,
     .intr_priority = 2,

    };
 
 //setup i2c master bus

 i2c_master_bus_handle_t bus_handle;
 ESP_ERROR_CHECK(i2c_new_master_bus(&master_config, &bus_handle));

 //i2c device config 

 i2c_device_config_t dev_cfg = {
    
     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
     .device_address =SLAVE_ADDR,
     .scl_speed_hz = I2C_MASTER_FREQ_HZ ,
      
    };

 // i2c device istance alloctaion ,adding i2c device to bus
 
 ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &device_handle));


} 

//=================================== data send ===========================================/

void Send_data_Task(void* Parameter){

 TickType_t xLastWakeTime = xTaskGetTickCount ();
 const TickType_t xFrequency = pdMS_TO_TICKS(1);
 union {
         int32_t convert_buf[DATA_LENGTH];
         uint8_t byte_buf[DATA_LENGTH*4]; 
        } byte_converter;

 int local_copy[NUM_ENCODERS];

 while(1){
     taskENTER_CRITICAL(&buf_mux);
        memcpy(local_copy,(void*)front_buf, sizeof(local_copy));
     taskEXIT_CRITICAL(&buf_mux);

     //process data

     ESP_LOGI(TAG,"Preparing to Send Data ");

     for (int i = 0 ; i<  DATA_LENGTH;i++){
       
         byte_converter.convert_buf[i] = local_copy[i];
       
        }
      // Sending_Data

      uint8_t  *DATA_TO_SEND = byte_converter.byte_buf;

      ESP_ERROR_CHECK(i2c_master_transmit(device_handle,DATA_TO_SEND,sizeof(byte_converter.byte_buf),-1));

      ESP_LOGI(TAG,"Yo!the Data is Sent:look down below to chek if the other guy rcieved it properly ");

      xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
