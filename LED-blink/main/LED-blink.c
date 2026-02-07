#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void blinkLed(void *pvValue ){

 char *Mytask = pcTaskGetName(NULL);

 ESP_LOGI(Mytask,"HELLO FROM Task1!\n");

 gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);
         
  while(1){

    
    gpio_set_level(GPIO_NUM_38,1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_38,0);
    vTaskDelay(6000/portTICK_PERIOD_MS);

     }
}

void app_main(void)
{

 xTaskCreate(blinkLed, "blink_task", 2048, NULL, 1, NULL);

}
