#include "robot_controller.h"

volatile DRAM_ATTR int32_t pcnt_count[NUM_MOTORS] __attribute__((aligned(4))) = {0};

portMUX_TYPE pcnt_mux = portMUX_INITIALIZER_UNLOCKED;


typedef struct{
	uint8_t PINA;
	uint8_t PINB;
}Encorder_pins;

Encorder_pins GPIOArray[NUM_MOTORS] = {
  {ENCORDER1_PINA,ENCORDER1_PINB},
  {ENCORDER2_PINA,ENCORDER2_PINB},
  {ENCORDER3_PINA,ENCORDER3_PINB},
  {ENCORDER4_PINA,ENCORDER4_PINB}
  
};

pcnt_unit_handle_t pcnt_unit[NUM_MOTORS]= {NULL,NULL,NULL,NULL};
 
QueueHandle_t encorderQue = NULL;

/****************************setting up PCNT units *************************/
void PCNT_Set(Encorder_pins *GPIO,uint8_t ID,pcnt_unit_handle_t *pcnt_unit ){
 
 gpio_set_pull_mode(GPIO->PINA,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(GPIO->PINB,GPIO_PULLDOWN_ONLY);
 
 ESP_LOGI(PCNT_TAG, "install pcnt unit%d",ID);
 pcnt_unit_config_t unit_config = {
     .high_limit = PCNT_HIGH_LIMIT,
     .low_limit = PCNT_LOW_LIMIT,
     .flags.accum_count = true ,
     .intr_priority = 0 // Medium priority
    };

 ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, pcnt_unit));

 // add high limit watch point
 ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*pcnt_unit, PCNT_HIGH_LIMIT));
 // add low limit watch point
 ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*pcnt_unit, PCNT_LOW_LIMIT));

 ESP_LOGI(PCNT_TAG, "set glitch filter for unit%d",ID);
 pcnt_glitch_filter_config_t filter_config = {
     .max_glitch_ns = 1000,
    };
 ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config));

 ESP_LOGI(PCNT_TAG, "install pcnt channels of unit%d",ID);
 pcnt_chan_config_t chan_a_config = {
     .edge_gpio_num = GPIO->PINA,
     .level_gpio_num = GPIO->PINB,
    };

 pcnt_channel_handle_t pcnt_chan_a = NULL;
 ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_a_config, &pcnt_chan_a));
 pcnt_chan_config_t chan_b_config = {
     .edge_gpio_num = GPIO->PINB,
     .level_gpio_num = GPIO->PINA,
    };
 pcnt_channel_handle_t pcnt_chan_b = NULL;
 ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_b_config, &pcnt_chan_b));

 ESP_LOGI(PCNT_TAG, "set edge and level actions for pcnt channels of unit%d",ID);
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

 ESP_LOGI(PCNT_TAG, "enable pcnt unit%d",ID);
    ESP_ERROR_CHECK(pcnt_unit_enable(*pcnt_unit));
    ESP_LOGI(PCNT_TAG, "clear pcnt unit%d",ID);
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
    ESP_LOGI(PCNT_TAG, "start pcnt unit%d",ID);
    ESP_ERROR_CHECK(pcnt_unit_start(*pcnt_unit));
}

/****************************READING COUNTER VALUES *************************/
void PCNT_Read_Task( void *pvParameters){

 TickType_t xLastWakeTime = xTaskGetTickCount ();
 const TickType_t xFrequency = pdMS_TO_TICKS(1000/PCNT_VAL_UPDATE_FREQ);
 
 ESP_LOGI(PCNT_TAG, "Read Task Stack: %u", uxTaskGetStackHighWaterMark(NULL));

 int32_t temp_counts[NUM_MOTORS];

 while(1){
     taskENTER_CRITICAL(&pcnt_mux);
     for (int i = 0;i<NUM_MOTORS;i++){
         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit[i], (int*)&temp_counts[i]));
        }
     taskEXIT_CRITICAL(&pcnt_mux);

     memcpy((void*)pcnt_count, temp_counts, sizeof(temp_counts));

     if(xQueueOverwrite(encorderQue,(void*)pcnt_count) != pdPASS) {
          ESP_LOGE(PCNT_TAG, "Queue overflow!");
        }
     
     xTaskDelayUntil( &xLastWakeTime, xFrequency );

     
    }
}

/****************************PRINTING TO CONSOLE *************************/
void PCNT_Print_Task(void *pvParameters){

 int32_t received_counts[NUM_MOTORS];
 for(;;){
      if(xQueueReceive(encorderQue,(int*)received_counts,10/portTICK_PERIOD_MS)){
         ESP_LOGI(PCNT_TAG,"\n"
                  "ENCODER[1] READING: %.2f\n"
                  "ENCODER[2] READING: %.2f\n"
                  "ENCODER[3] READING: %.2f\n"
                  "ENCODER[4] READING: %.2f",
                  (float)received_counts[0] / EN_PPR,
                  (float)received_counts[1] / EN_PPR,
                  (float)received_counts[2] / EN_PPR,
                  (float)received_counts[3] / EN_PPR);
        }
     vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

/****************************Main App *************************/
void pcnt_app_main(void){
 ESP_LOGI(PCNT_TAG, "setting up PCNT units and chaneels\n" );
 for (int i = 0;i<NUM_MOTORS;i++){
     PCNT_Set(&GPIOArray[i],i+1,&pcnt_unit[i]); 
    }
 
 ESP_LOGI(PCNT_TAG, "setting up Ques" );
 encorderQue = xQueueCreate(1, sizeof(pcnt_count));

 ESP_LOGI(PCNT_TAG, "setting up Tasks" );     
 TaskHandle_t xHandle1 = NULL;
 xTaskCreatePinnedToCore(
                    PCNT_Read_Task,      /* Function that implements the task. */
                    "Read_Task",             /* Text name for the task. */
                    4096,            /* Stack size in bytess. */
                    NULL,            /* Parameter passed into the task. */
                    (24|portPRIVILEGE_BIT),               /* Priority at which the task is created. */
                    &xHandle1,       /* Used to pass out the created task's handle. */
                    1 );     //core affinity

 TaskHandle_t xHandle2 = NULL;
 xTaskCreatePinnedToCore(
                    PCNT_Print_Task,      /* Function that implements the task. */
                    "print_Task",             /* Text name for the task. */
                    4096,            /* Stack size in bytes. * */
                    NULL,            /* Parameter passed into the task. */
                    5,               /* Priority at which the task is created. */
                    &xHandle2,       /* Used to pass out the created task's handle. */
                    tskNO_AFFINITY );     //core affinity

 ESP_LOGI(PCNT_TAG, "Task Initialization Complete. Tasks running.");

 ESP_LOGI(PCNT_TAG, "PCNT unit setiing up completed\n");

}

