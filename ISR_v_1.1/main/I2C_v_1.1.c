#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

 

extern DRAM_ATTR int16_t ENCORDER_COUNT[8];


extern void PIN_setup();
extern void INTERRUPT_SET();

#define I2C_MASTER_SCL_IO          18
#define I2C_MASTER_SDA_IO          17
#define I2C_PORT_NUM               I2C_NUM_0   // Using I2C port 0
#define I2C_MASTER_FREQ_HZ         100000      // I2C clock frequency (100 kHz)
#define I2C_MASTER_GLITCH_FILTER   7           // Glitch filter threshold

#define SLAVE_ADDR                 0x77        // 7-bit I2C address of the slave device
#define DATA_LENGTH                8
i2c_master_dev_handle_t device_handle;

void I2C_Master_setup();
void Send_data( int16_t encorder_read[DATA_LENGTH]);


void app_main(void)
{
  //setup io pins
 PIN_setup();
 //setup interuupts 
 INTERRUPT_SET();

 //set master 
 I2C_Master_setup() ;

while(1){
 for (int i = 0 ;i < 8 ;i ++){
 ESP_LOGI("master","ENCORDER[%d]) READING: %d \n ",i+1 ,ENCORDER_COUNT[i]);
 }

 vTaskDelay(100/portTICK_PERIOD_MS);
}



 //send data to slave 
 //Send_data(ENCORDER_COUNT);
}

/***** Master Setup *****/

void I2C_Master_setup(void){

 //i2c masterBus Config

 i2c_master_bus_config_t master_config={
     
     .clk_source = I2C_CLK_SRC_DEFAULT,
     .i2c_port = I2C_PORT_NUM,
     .scl_io_num = I2C_MASTER_SCL_IO,
     .sda_io_num = I2C_MASTER_SDA_IO,
     .glitch_ignore_cnt = 7,
     .flags.enable_internal_pullup = true,
     .intr_priority = 2,

    };
 
 //setup i2c master bus

 i2c_master_bus_handle_t bus_handle;
 ESP_ERROR_CHECK(i2c_new_master_bus(&master_config, &bus_handle));

 //i2c device config 

 i2c_device_config_t dev_cfg = {
    
     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
     .device_address =SLAVE_ADDR,
     .scl_speed_hz = 100000,
      
    };

 // i2c device istance alloctaion ,adding i2c device to bus
 //i2c_master_dev_handle_t device_handle;
 ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &device_handle));


} 

/****** data send ******/

void Send_data(int16_t encorder_read[DATA_LENGTH]){

 //process data

 union {
      int16_t convert_buf[DATA_LENGTH];
      uint8_t byte_buf[DATA_LENGTH*2]; 
     } byte_converter;

 

 ESP_LOGI("MASTER","Preparing to Send Data ");

 for (int i = 0 ; i<  DATA_LENGTH;i++){
       
       byte_converter.convert_buf[i] = encorder_read[i];
       
    }
 // Sending_Data

 uint8_t  *DATA_TO_SEND = byte_converter.byte_buf;
 ESP_ERROR_CHECK(i2c_master_transmit(device_handle,DATA_TO_SEND,18,-1));
    
    
 
 ESP_LOGI("MASTER","Yo!the Data is Sent:look down below to chek if the other guy rcieved it properly ");

}