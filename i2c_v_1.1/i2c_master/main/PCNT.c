#include "Decorder.h"
static const char *TAG = "PCNT";

#define GPIO_MODE GPIO_FLOATING

#define PCNT_HIGH_LIMIT 32000
#define PCNT_LOW_LIMIT -32000

#define ENCORDER1_PINA 32
#define ENCORDER1_PINB 33

#define ENCORDER2_PINA 25
#define ENCORDER2_PINB 26

#define ENCORDER3_PINA 27
#define ENCORDER3_PINB 14

#define ENCORDER4_PINA 23
#define ENCORDER4_PINB 22

#define ENCORDER5_PINA 21
#define ENCORDER5_PINB 19

#define ENCORDER6_PINA 18
#define ENCORDER6_PINB 5

#define ENCORDER7_PINA 17
#define ENCORDER7_PINB 16

#define ENCORDER8_PINA 4
#define ENCORDER8_PINB 2

volatile DRAM_ATTR int32_t pcnt_buf_A[NUM_ENCODERS];
volatile DRAM_ATTR int32_t pcnt_buf_B[NUM_ENCODERS];

volatile DRAM_ATTR int32_t *front_buf = pcnt_buf_A;   // readers use this
volatile DRAM_ATTR int32_t *back_buf  = pcnt_buf_B;   // writer updates this

portMUX_TYPE buf_mux = portMUX_INITIALIZER_UNLOCKED;

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


pcnt_unit_handle_t pcnt_unit[NUM_ENCODERS]= {NULL};

QueueHandle_t encorderQue = NULL;
 
//====================================================setting up PCNT units ========================================/
void PCNT_Set(Encorder_pins *GPIO,uint8_t ID,pcnt_unit_handle_t *pcnt_unit ){

 gpio_reset_pin(GPIO->PINA);
 gpio_reset_pin(GPIO->PINB);
 gpio_set_pull_mode(GPIO->PINA,GPIO_MODE);
 gpio_set_pull_mode(GPIO->PINB,GPIO_MODE);
 
 ESP_LOGI(TAG, "install pcnt unit%d",ID);
 pcnt_unit_config_t unit_config = {
     .high_limit = PCNT_HIGH_LIMIT,
     .low_limit = PCNT_LOW_LIMIT,
     .flags.accum_count = true,
     .intr_priority = 3 // Medium priority
    };

 ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, pcnt_unit));

 ESP_LOGI(TAG, "set glitch filter for unit%d",ID);
 pcnt_glitch_filter_config_t filter_config = {
     .max_glitch_ns = 2000,
    };
 ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config));

 // add high limit watch point
 ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*pcnt_unit, PCNT_HIGH_LIMIT));
 // add low limit watch point
 ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*pcnt_unit, PCNT_LOW_LIMIT));

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

static inline void swap_buffers(void) {
    taskENTER_CRITICAL(&buf_mux);
    int32_t *tmp = (void*)front_buf;
    front_buf = back_buf;
    back_buf  = tmp;
    taskEXIT_CRITICAL(&buf_mux);
}


//========================================READING COUNTER VALUES ======================================/
void PCNT_Read_Task( void *pvParameters){

 TickType_t xLastWakeTime = xTaskGetTickCount ();
 const TickType_t xFrequency = pdMS_TO_TICKS(1);
 //BaseType_t xWasDelayed;

 static DRAM_ATTR int32_t PCNT_Buff[NUM_ENCODERS]={0};
 int count_value;

 while(1){

     for (int i = 0; i < NUM_ENCODERS; i++){
         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit[i],&count_value));
         back_buf[i] =(int32_t)count_value;
        }

     for(int i = 0;i<NUM_ENCODERS; i++){
          PCNT_Buff[i] = back_buf[i];
        }

     swap_buffers();

      xQueueOverwrite(encorderQue,(int*)PCNT_Buff);
     
      xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

//======================================PRINTING TO CONSOLE =========================================/
void PCNT_Print_Task(void *Parameters){

 int32_t Data_buff[NUM_ENCODERS] = {0};

while(1){

    if (xQueueReceive(encorderQue, Data_buff, portMAX_DELAY) == pdTRUE){
         ESP_LOGI(TAG," \n"
                  "ENCODER[1] READING: %" PRId32 "\n"
                  "ENCODER[2] READING: %" PRId32 "\n"
                  "ENCODER[3] READING: %" PRId32 "\n"
                  "ENCODER[4] READING: %" PRId32 "\n"
                  "ENCODER[5] READING: %" PRId32 "\n"
                  "ENCODER[6] READING: %" PRId32 "\n"
                  "ENCODER[7] READING: %" PRId32 "\n"
                  "ENCODER[8] READING: %" PRId32,
                   Data_buff[0], Data_buff[1],
                   Data_buff[2], Data_buff[3],
                   Data_buff[4], Data_buff[5],
                   Data_buff[6], Data_buff[7]);
    }
       
 vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

//==================================Main App =================================================/
void PCNT_app_main(void){
 ESP_LOGI(TAG, "setting up PCNT units and chanels" );
 for (int i = 0;i<NUM_ENCODERS;i++){
     PCNT_Set(GPIO_PointerArray[i],i+1,&pcnt_unit[i]); 
    }

encorderQue = xQueueCreate(1,sizeof(int32_t)*NUM_ENCODERS);
 if (encorderQue == NULL) {
    ESP_LOGE(TAG, "Failed to create encoder queue!");
}
 
 ESP_LOGI(TAG, "PCNT unit setiing up completed\n");
 
 ESP_LOGI(TAG, "setting up   pcnt Que" );
 //encorderQue = xQueueCreate(15,sizeof(int)*NUM_ENCORDERS);

 ESP_LOGI(TAG, "setting up PCNT Read_Tasks" );
 TaskHandle_t xHandle1 = NULL;
 xTaskCreatePinnedToCore(
                    PCNT_Read_Task,      /* Function that implements the task. */
                    "Read_Task",             /* Text name for the task. */
                    5120,            /* Stack size in bytess. */
                    NULL,            /* Parameter passed into the task. */
                    (20),               /* Priority at which the task is created. */
                    &xHandle1,       /* Used to pass out the created task's handle. */
                    0 );     //core affinity

 TaskHandle_t Read_task = NULL;
 xTaskCreatePinnedToCore( PCNT_Print_Task,      // Function that implements the task. 
                          "print_Task",             // Text name for the task. 
                          3072,            // Stack size in bytes.
                          NULL,
                          10,            // Priority at which the task is created. 
                         &Read_task,       // Used to pass out the created task's handle. 
                         tskNO_AFFINITY );     //core affinity


 ESP_LOGI(TAG, "PCNT Task Initialization Complete. Tasks running.");

}

