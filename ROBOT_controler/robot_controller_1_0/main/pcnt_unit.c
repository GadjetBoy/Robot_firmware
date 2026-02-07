#include "robot_controller.h"

volatile DRAM_ATTR int Data_Buff[NUM_ENCORDERS] = {0};

static const char *TAG = "PCNT";
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


typedef struct{
	uint8_t PINA;
	uint8_t PINB;
}Encorder_pins;

Encorder_pins GPIOArray[] = {
  {ENCORDER1_PINA,ENCORDER1_PINB},
  {ENCORDER2_PINA,ENCORDER2_PINB},
  {ENCORDER3_PINA,ENCORDER3_PINB},
  {ENCORDER4_PINA,ENCORDER4_PINB},
  {ENCORDER5_PINA,ENCORDER5_PINB},
  {ENCORDER6_PINA,ENCORDER6_PINB},
  {ENCORDER7_PINA,ENCORDER7_PINB},
  {ENCORDER8_PINA,ENCORDER8_PINB} 
};

Encorder_pins *GPIO_PointerArray[] = {&GPIOArray[0],
                                      &GPIOArray[1],
                                      &GPIOArray[2], 
                                      &GPIOArray[3],
                                      &GPIOArray[4],
                                      &GPIOArray[5],
                                      &GPIOArray[6],
                                      &GPIOArray[7]};


pcnt_unit_handle_t pcnt_unit[NUM_ENCORDERS]= {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
 
QueueHandle_t encorderQue = NULL;

/****************************setting up PCNT units *************************/
void PCNT_Set(Encorder_pins *GPIO,uint8_t ID,pcnt_unit_handle_t *pcnt_unit ){
 
 gpio_set_pull_mode(GPIO->PINA,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(GPIO->PINB,GPIO_PULLDOWN_ONLY);
 
 ESP_LOGI(TAG, "install pcnt unit%d",ID);
 pcnt_unit_config_t unit_config = {
     .high_limit = PCNT_HIGH_LIMIT,
     .low_limit = PCNT_LOW_LIMIT,
     .flags.accum_count = true,
    };

 ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, pcnt_unit));

 ESP_LOGI(TAG, "set glitch filter for unit%d",ID);
 pcnt_glitch_filter_config_t filter_config = {
     .max_glitch_ns = 1000,
    };
 ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config));

 ESP_LOGI(TAG, "install pcnt channels of unit%d",ID);
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

 ESP_LOGI(TAG, "set edge and level actions for pcnt channels of unit%d",ID);
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

 ESP_LOGI(TAG, "enable pcnt unit%d",ID);
    ESP_ERROR_CHECK(pcnt_unit_enable(*pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit%d",ID);
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit%d",ID);
    ESP_ERROR_CHECK(pcnt_unit_start(*pcnt_unit));
}

/****************************READING COUNTER VALUES *************************/
void PCNT_Read_Task( void *pvParameters){

 TickType_t xLastWakeTime = xTaskGetTickCount ();
 const TickType_t xFrequency = pdMS_TO_TICKS(1000/PCNT_VAL_UPDATE_FREQ);
 //BaseType_t xWasDelayed;

 

 while(1){
     taskENTER_CRITICAL(&mux);
     for (int a = 0;a<NUM_ENCORDERS;a++){
         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit[a], (int*)&Data_Buff[a]));
        }
     taskEXIT_CRITICAL(&mux); 
     xQueueOverwrite(encorderQue,(int*)Data_Buff);
     //vTaskDelay(pdMS_TO_TICKS(1));
     xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************PRINTING TO CONSOLE *************************/
void PCNT_Print_Task(void *pvParameters){

 int received_counts[NUM_ENCORDERS];
 for(;;){
      if(xQueuePeek(encorderQue,received_counts,10/portTICK_PERIOD_MS)){
         ESP_LOGI(TAG," \n"
         	      "ENCODER[1] READING: %d\n"
                  "ENCODER[2] READING: %d\n"
                  "ENCODER[3] READING: %d\n"
                  "ENCODER[4] READING: %d\n"
                  "ENCODER[5] READING: %d\n"
                  "ENCODER[6] READING: %d\n"
                  "ENCODER[7] READING: %d\n"
                  "ENCODER[8] READING: %d",
                   received_counts[0], received_counts[1],
                   received_counts[2], received_counts[3],
                   received_counts[4], received_counts[5],
                   received_counts[6], received_counts[7]);
        }
     vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

/****************************Main App *************************/
void pcnt_app_main(void){
 ESP_LOGI(TAG, "setting up PCNT units and chaneels" );
 for (int i = 0;i<NUM_ENCORDERS;i++){
     PCNT_Set(GPIO_PointerArray[i],i+1,&pcnt_unit[i]); 
    }
 
 ESP_LOGI(TAG, "PCNT unit setiing up completed\n");
 
 ESP_LOGI(TAG, "setting up Ques" );
 encorderQue = xQueueCreate(1,sizeof(int)*NUM_ENCORDERS);

 ESP_LOGI(TAG, "setting up Tasks" );
 TaskHandle_t xHandle1 = NULL;
 xTaskCreatePinnedToCore(
                    PCNT_Read_Task,      /* Function that implements the task. */
                    "Read_Task",             /* Text name for the task. */
                    2048,            /* Stack size in bytess. */
                    NULL,            /* Parameter passed into the task. */
                    (24|portPRIVILEGE_BIT),               /* Priority at which the task is created. */
                    &xHandle1,       /* Used to pass out the created task's handle. */
                    1 );     //core affinity

 TaskHandle_t xHandle2 = NULL;
 xTaskCreatePinnedToCore(
                    PCNT_Print_Task,      /* Function that implements the task. */
                    "print_Task",             /* Text name for the task. */
                    3072,            /* Stack size in bytes. * */
                    NULL,            /* Parameter passed into the task. */
                    1,               /* Priority at which the task is created. */
                    &xHandle2,       /* Used to pass out the created task's handle. */
                    tskNO_AFFINITY );     //core affinity

 ESP_LOGI(TAG, "Task Initialization Complete. Tasks running.");

}

