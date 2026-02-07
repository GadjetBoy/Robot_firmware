#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO          18
#define I2C_MASTER_SDA_IO          17
#define I2C_PORT_NUM               I2C_NUM_0   // Using I2C port 0
#define I2C_MASTER_FREQ_HZ         100000      // I2C clock frequency (100 kHz)
#define I2C_MASTER_GLITCH_FILTER   7           // Glitch filter threshold

#define SLAVE_ADDR                 0x78        // 7-bit I2C address of the slave device



void app_main(void)
{
 //master Port Config
 i2c_master_bus_config_t Master_config = {
     
     .clk_source = I2C_CLK_SRC_DEFAULT,
     .i2c_port = I2C_PORT_NUM,
     .scl_io_num = I2C_MASTER_SCL_IO,
     .sda_io_num = I2C_MASTER_SDA_IO,
     .glitch_ignore_cnt = 7,
     .flags.enable_internal_pullup = true,
     .intr_priority = 1,

     };	

 // master port allocation
 i2c_master_bus_handle_t bus_Handle;
 ESP_ERROR_CHECK( i2c_new_master_bus(&Master_config,&bus_Handle));

 //device config
  i2c_device_config_t Slave_config ={

  	 .dev_addr_length = I2C_ADDR_BIT_LEN_7,
  	 .device_address = SLAVE_ADDR ,
  	 .scl_speed_hz = 100000,

     };

 // i2c device istance alloctaion ,addind i2c device to bus
 i2c_master_dev_handle_t dev_handle;
 ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_Handle,&Slave_config,&dev_handle));

 // data to SEND 

 const char *DATA_TO_SEND = "hello from master";
 size_t DATA_LENGTH = (sizeof("hello from master")-1);

 ESP_ERROR_CHECK(i2c_master_transmit(dev_handle,(uint8_t*)DATA_TO_SEND,DATA_LENGTH,-1));

 ESP_LOGI("MASTER","Yo!the Message %s is Sent:look down below to chek if the other guy rcieved it properly ",DATA_TO_SEND);


 }
