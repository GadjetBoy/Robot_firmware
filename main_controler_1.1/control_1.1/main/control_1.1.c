#include "control_1.1.h"

#define I2C_PORT                 I2C_NUM_0  
#define I2C_SLAVE_SCL_IO         15
#define I2C_SLAVE_SDA_IO         13
#define SLAVE_ADDR               0x77

#define TAG "I2C SLAVE"
#define DATA_LENGTH              32
#define QUEUE_LENGTH             15
#define GPIO_NUM                 12

#define i2c_log_freq             100

typedef struct {
     uint8_t rx_data[DATA_LENGTH];
     size_t len;
} i2c_message_t;

DRAM_ATTR i2c_message_t rx_msg ;

typedef struct {
     i2c_slave_dev_handle_t slave_handle;
     QueueHandle_t recieve_queue;
     bool error;
}i2c_slave_context_t;


volatile int encorder_val[NUM_MOTORS] = {0};

void I2C_Salve_Device_Setup(i2c_slave_context_t *context);

static bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *evt_data, void *user_data);
void Data_Recive_task(void* arg);
void PCNT_Print_Task(void *Parameters);
void blinkLed(void *pvValue );

TaskHandle_t Recieve_task = NULL;
TaskHandle_t Print_task = NULL;
TaskHandle_t blinkTaskHandle = NULL;
QueueHandle_t encorderQue = NULL;


portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;
//=========================================== app main =====================================================
void app_main(void)
{
 gpio_set_direction(GPIO_NUM, GPIO_MODE_OUTPUT);
         
 static i2c_slave_context_t context = {0};

 context.recieve_queue = xQueueCreate(QUEUE_LENGTH,sizeof(i2c_message_t));
  if (!context.recieve_queue) {
        ESP_LOGE(TAG, "Creating queue failed");     
        return;
    }
 context.error = false;

 //setting up the LED 
 gpio_reset_pin(GPIO_NUM);                     // Reset pin state
 gpio_set_direction(GPIO_NUM, GPIO_MODE_OUTPUT);
 gpio_set_level(GPIO_NUM, 0);

 // setting up salave  
 I2C_Salve_Device_Setup(&context);

 ESP_LOGI(TAG, "Slave initialization complete and ready to receive data");

  
  xTaskCreatePinnedToCore( Data_Recive_task,      /* Function that implements the task. */
                          "I2C_recieve_Task",     /* Text name for the task. */
                           5120,                  /* Stack size in bytes. * */
                           &context,                  /* Parameter passed into the task. */
                           23,                     /* Priority at which the task is created. */
                           &Recieve_task,         /* Used to pass out the created task's handle. */
                           1);      //core affinity

  
  xTaskCreatePinnedToCore( PCNT_Print_Task,      /* Function that implements the task. */
                          "Print_task",     /* Text name for the task. */
                           3072,                  /* Stack size in bytes. * */
                           NULL,                  /* Parameter passed into the task. */
                           5,                     /* Priority at which the task is created. */
                           &Print_task,         /* Used to pass out the created task's handle. */
                           tskNO_AFFINITY );      //core affinity

  
  xTaskCreatePinnedToCore( blinkLed,      /* Function that implements the task. */
                          "blink_task",     /* Text name for the task. */
                           1024,                  /* Stack size in bytes. * */
                           NULL,                  /* Parameter passed into the task. */
                           10,                     /* Priority at which the task is created. */
                           &blinkTaskHandle,         /* Used to pass out the created task's handle. */
                           tskNO_AFFINITY );

  ESP_LOGI(TAG, "Setting up PWM and PID module ");

  pwm_app_main();
  pid_app_main(); 
  mot_command_app_main();
  BLE_app_main();

}

//=======================================I2C Slave Device Setup ====================================================/

void I2C_Salve_Device_Setup(i2c_slave_context_t *context){
   
 //configure slave 
 i2c_slave_config_t i2c_slv_config = {
     .addr_bit_len = I2C_ADDR_BIT_LEN_7,
     .clk_source = I2C_CLK_SRC_DEFAULT,
     .i2c_port = I2C_PORT,
     .send_buf_depth = DATA_LENGTH ,
     .receive_buf_depth = DATA_LENGTH ,
     .scl_io_num = I2C_SLAVE_SCL_IO,
     .sda_io_num = I2C_SLAVE_SDA_IO,
     .slave_addr =SLAVE_ADDR,
     .intr_priority = 3,
    };

 ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config,&(context->slave_handle)));

 i2c_slave_event_callbacks_t Call_back = {
     .on_receive  =  i2c_slave_rx_done_callback,
    };

 //Registering CallBack Function
  ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(context->slave_handle, &Call_back,context));

}
//================================================= callback function =============================================================/

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *evt_data, void *user_data){
    
    i2c_slave_context_t *context = (i2c_slave_context_t *)user_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    memcpy(rx_msg.rx_data, evt_data->buffer, DATA_LENGTH);

    // Try to send message into queue
    if (xQueueSendFromISR(context->recieve_queue, &rx_msg, &xHigherPriorityTaskWoken) != pdTRUE) {
        // Queue is full → data will be lost
         context->error = true ;
    }
    // Notify the recieve task (increment its notification value)
    vTaskNotifyGiveFromISR(Recieve_task, &xHigherPriorityTaskWoken);
    // Notify LED Task
    vTaskNotifyGiveFromISR(blinkTaskHandle, &xHigherPriorityTaskWoken);

    return (xHigherPriorityTaskWoken==pdTRUE);
}

//================================================== Recieve Data from Master======================================================/

void IRAM_ATTR Data_Recive_task(void* arg){
    i2c_slave_context_t *context = (i2c_slave_context_t *) arg;
    i2c_message_t msg;

    static int log_counter = 0;

    while(1){
        // Block until at least one notification, get count, clear to 0
        uint32_t notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        log_counter++;

        if(log_counter>=i2c_log_freq){
          ESP_LOGI(TAG, "Receiving ..... (notify count: %lu)\n", notify_count);
        }

        for (uint32_t i = 0; i < notify_count; i++) {
            if (xQueueReceive(context->recieve_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (context->error == true) {
                    ESP_LOGE(TAG, "RX queue overflow detected earlier\n");
                    context->error = false;
                }
                taskENTER_CRITICAL(&encoder_mux);
                memcpy((void *)encorder_val, msg.rx_data, DATA_LENGTH);
                taskEXIT_CRITICAL(&encoder_mux);

             if(log_counter>=i2c_log_freq){
                 ESP_LOGI(TAG, "Data Received - Encoder values updated\n");
                 log_counter = 0;
                }
                
                  
            } else {
                ESP_LOGE(TAG, "Queue receive failed (possible mismatch or timeout)\n");
            }
        }

     vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void PCNT_Print_Task(void *Parameters){

 while(1){
         // Use the safe function to read all encoder values
         int enc_values[NUM_MOTORS];
         for (int i = 0; i < NUM_MOTORS; i++) {
              enc_values[i] = safe_read_encoder(i);
            }
         /*
         // Atomically update BLE buffer
          taskENTER_CRITICAL(&ble_mux);
          for (int i = 0; i < NUM_MOTORS; i++) {
             tx_sensor_data[i] = enc_values[i];
            }
          taskEXIT_CRITICAL(&ble_mux);*/
           
         ESP_LOGI(TAG," \n"
                  "ENCODER[1] READING: %d\n"
                  "ENCODER[2] READING: %d\n"
                  "ENCODER[3] READING: %d\n"
                  "ENCODER[4] READING: %d\n"
                  "ENCODER[5] READING: %d\n"
                  "ENCODER[6] READING: %d\n"
                  "ENCODER[7] READING: %d\n"
                  "ENCODER[8] READING: %d",
                   enc_values[0], enc_values[1],
                   enc_values[2], enc_values[3],
                   enc_values[4], enc_values[5],
                   enc_values[6], enc_values[7]);

         ESP_LOGI(TAG,".............\n");

         vTaskDelay(pdMS_TO_TICKS(100)); 
        }
     
}

void blinkLed(void *pvValue ){

  while(1){
     ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

     gpio_set_level(GPIO_NUM,1);
     vTaskDelay(50/portTICK_PERIOD_MS);
     gpio_set_level(GPIO_NUM,0);
     vTaskDelay(50/portTICK_PERIOD_MS);

    }
}