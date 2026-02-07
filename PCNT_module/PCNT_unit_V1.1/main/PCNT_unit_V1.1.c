#include <stdio.h>
#include "driver/pulse_cnt.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "PCNT";

#define PCNT_HIGH_LIMIT 20000
#define PCNT_LOW_LIMIT -20000

#define ENCORDER1_PINA 1
#define ENCORDER2_PINA 2
#define ENCORDER3_PINA 3
#define ENCORDER4_PINA 4

#define ENCORDER1_PINB 9
#define ENCORDER2_PINB 10
#define ENCORDER3_PINB 11
#define ENCORDER4_PINB 12

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED; // Statically allocate and initialize the spinlock

/*struct Encorder_Pins{
  PINA;
  PINB;      // conventional way to define a structure
   }; 

struct Encorder_pins E_pins[] = {};*/ 

typedef struct{
	uint8_t PINA;
	uint8_t PINB;
}Encorder_pins;

Encorder_pins GPIOArray[] = {
  {ENCORDER1_PINA,ENCORDER1_PINB},
  {ENCORDER2_PINA,ENCORDER2_PINB},
  {ENCORDER3_PINA,ENCORDER3_PINB},
  {ENCORDER4_PINA,ENCORDER4_PINB}
};

#define NUM_ENCORDERS (sizeof(GPIOArray)/sizeof(GPIOArray[0]))

volatile DRAM_ATTR int Data_Buff[NUM_ENCORDERS] = {0};

pcnt_unit_handle_t pcnt_unit[NUM_ENCORDERS]= {NULL, NULL, NULL, NULL};



void PCNT_Set(Encorder_pins GPIO,uint8_t ID,pcnt_unit_handle_t *pcnt_unit ){

 ESP_LOGI(TAG, "install pcnt unit%d",ID);
 pcnt_unit_config_t unit_config = {
     .high_limit = PCNT_HIGH_LIMIT,
     .low_limit = PCNT_LOW_LIMIT,
     //.accum_count = true,
    };


 ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, pcnt_unit));

 ESP_LOGI(TAG, "set glitch filter for unit%d",ID);
 pcnt_glitch_filter_config_t filter_config = {
     .max_glitch_ns = 1000,
    };
 ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config));

 ESP_LOGI(TAG, "install pcnt channels of unit%d",ID);
 pcnt_chan_config_t chan_a_config = {
     .edge_gpio_num = GPIO.PINA,
     .level_gpio_num = GPIO.PINB,
    };

 pcnt_channel_handle_t pcnt_chan_a = NULL;
 ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_a_config, &pcnt_chan_a));
 pcnt_chan_config_t chan_b_config = {
     .edge_gpio_num = GPIO.PINB,
     .level_gpio_num = GPIO.PINA,
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

void PCNT_VAL_Read_Task( void *pvParameters){

 for(;;){
     taskENTER_CRITICAL(&my_spinlock);
     for (int a = 0;a<NUM_ENCORDERS;a++){
         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit[a], (int*)&Data_Buff[a]));
        }
     taskEXIT_CRITICAL(&my_spinlock);
     vTaskDelay(pdMS_TO_TICKS(10));
    }

}

void Data_Print_Out_Task(void *pvParameters){

char buf[256];
 for(;;){
      taskENTER_CRITICAL(&my_spinlock);
      snprintf(buf, sizeof(buf),
         "\n"
         "ENCORDER[1]) READING: %d\n"
         "\n"
         "ENCORDER[2]) READING: %d\n"
         "\n"
         "ENCORDER[3]) READING: %d\n"
         "\n"
         "ENCORDER[4]) READING: %d\n"
         "\n"
         "\n",
         Data_Buff[0],Data_Buff[1],Data_Buff[2],Data_Buff[3]);

     taskEXIT_CRITICAL(&my_spinlock);
     ESP_LOGI(TAG, "%s", buf);
     vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void app_main(void){
 ESP_LOGI(TAG, "setting up PCNT units and chaneels" );
 for (int i = 0;i<NUM_ENCORDERS;i++){
     PCNT_Set(GPIOArray[i],i+1,&pcnt_unit[i]);
    }
 
 ESP_LOGI(TAG, "setting up Tasks" );
 TaskHandle_t xHandle1 = NULL;
 xTaskCreatePinnedToCore(
                    PCNT_VAL_Read_Task,      /* Function that implements the task. */
                    "Read_Task",             /* Text name for the task. */
                    2048,            /* Stack size in words, not bytes. */
                    NULL,            /* Parameter passed into the task. */
                    (3|portPRIVILEGE_BIT),               /* Priority at which the task is created. */
                    &xHandle1,       /* Used to pass out the created task's handle. */
                    tskNO_AFFINITY );     //core affinity

 TaskHandle_t xHandle2 = NULL;
 xTaskCreatePinnedToCore(
                    Data_Print_Out_Task,      /* Function that implements the task. */
                    "print_Task",             /* Text name for the task. */
                    3072,            /* Stack size in words, not bytes. */
                    NULL,            /* Parameter passed into the task. */
                    2,               /* Priority at which the task is created. */
                    &xHandle2,       /* Used to pass out the created task's handle. */
                    tskNO_AFFINITY );     //core affinity


}

