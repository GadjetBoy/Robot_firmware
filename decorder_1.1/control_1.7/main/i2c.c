#include "control_1.7.h"
#include "pcnt.h"
#include "cpg.h"
#include "ble.h"
#include "UART.h"

static const char *TAG = "I2C MASTER";

#define I2C_MASTER_SCL_IO          13
#define I2C_MASTER_SDA_IO          15
#define I2C_PORT_NUM               I2C_NUM_0   // Using I2C port 0
#define I2C_MASTER_FREQ_HZ         400000      // I2C clock frequency (400 kHz)
#define I2C_MASTER_GLITCH_FILTER   7           // Glitch filter threshold

#define SLAVE_ADDR                 0x77        // 7-bit I2C address of the slave device
#define DATA_LENGTH                8
#define LED_PIN                    12

#define I2C_UPDATE_RATE_HZ 10000 //HZ
#define I2C_DT (1.0f / I2C_UPDATE_RATE_HZ)  //seconds
#define I2C_UPDATE_RATE_US I2C_DT*1000000.0f // microseconds

i2c_master_dev_handle_t device_handle;

// Timer handle for high-frequency i2c updates
TaskHandle_t i2c_send_task = NULL;
static esp_timer_handle_t i2c_timer_handle;
portMUX_TYPE pid_buffer_safe_mux;

void I2C_Master_setup();
void Send_data_Task( void* Parameter);
void Blink_Task(void* Parameter);
void PCNT_Print_Task(void *Parameters);

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
     .intr_priority = 3,

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

void IRAM_ATTR i2c_timer_callback(void* arg) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send notification to i2c task
    vTaskNotifyGiveFromISR(i2c_send_task, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void read_PID_commands(float *dest) {
    uint8_t read_buffer;
    
    taskENTER_CRITICAL(&pid_buffer_swap_mux);
    read_buffer = current_write_index;
    
    
    memcpy(dest,(void*) motor_cmd_buffer[read_buffer], sizeof(float) * NUM_MOTORS);
    
    taskEXIT_CRITICAL(&pid_buffer_swap_mux);
}

//=================================== data send ===========================================/

void Send_data_Task(void* Parameter){

 union {
         float convert_buf[NUM_MOTORS];
         uint8_t byte_buf[NUM_MOTORS*4]; 
        } byte_converter;

 while(1){

 	 ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

 	 read_PID_commands(byte_converter.convert_buf);

 	 /*for(int i=0;i<NUM_MOTORS;i++){
         ESP_LOGE(TAG, "I2C_IN[%1d]: %.2f)",i ,byte_converter.convert_buf[i]);
        }*/

     //process data

     //ESP_LOGI(TAG,"Preparing to Send Data ");
       
      // Sending_Data
      
      uint8_t  *data_to_send = byte_converter.byte_buf;

      ESP_ERROR_CHECK(i2c_master_transmit(device_handle,data_to_send,sizeof(byte_converter.byte_buf),-1));

      //ESP_LOGI(TAG,"send Complete");
    }
}

void i2c_hardware_Timer_setup(void){
    // Setup high-resolution timer for 10 kHz
    esp_timer_create_args_t timer_args = {
        .callback = &i2c_timer_callback,  // Function to call
        .arg = NULL,                      // Argument passed to callback
        .dispatch_method = ESP_TIMER_TASK, // How to dispatch callback
        .name = "i2c_timer"               // Timer name for debugging
    };

    if (esp_timer_create(&timer_args, &i2c_timer_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create i2c timer!");
        return;
    }

    // Start the 10 kHz timer
    if (esp_timer_start_periodic(i2c_timer_handle, I2C_UPDATE_RATE_US) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start i2c timer!");
        return;
    }
}

void app_main(void){
 
 ESP_LOGI(TAG, "setting up I2C Send_Tasks" );

 ESP_LOGI(TAG, "setting up I2C MASTER device" );
 I2C_Master_setup() ;
 ESP_LOGI(TAG, "setting up I2C MASTER device Complete");

 xTaskCreatePinnedToCore(Send_data_Task, 
                         "Send_task", 
                          5119,
                          NULL,                //(void*)Data_buff, 
                          20,
                          &i2c_send_task,
                          0);

gpio_reset_pin(LED_PIN);
gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); 
gpio_set_level(LED_PIN, 1);
vTaskDelay(pdMS_TO_TICKS(500));
gpio_set_level(LED_PIN, 0);

ESP_LOGI(TAG, "setting up I2C MASTER device finish" );


 PCNT_app_main();
 vTaskDelay(pdMS_TO_TICKS(10));
 UART_app_main();
 vTaskDelay(pdMS_TO_TICKS(10));
 pid_app_main();
 vTaskDelay(pdMS_TO_TICKS(2));
 CPG_app_main();
 //i2c_hardware_Timer_setup();
 BLE_app_main();
 vTaskDelay(pdMS_TO_TICKS(10));

 esp_reset_reason_t reason = esp_reset_reason();
 ESP_LOGE("RESET", "CPU Reset reason: %d", reason);

 ESP_LOGI(TAG, "decorder initialization complete");

 

 
}


