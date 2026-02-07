#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_38

void blinkLed(void *pvParameter) {
    (void)pvParameter; // Mark parameter as unused
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); 
    while(1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
   
    xTaskCreate(blinkLed, "blink_task", 2048, NULL, 1, NULL);
    
    ESP_LOGI("MAIN_TASK", "HELLO FROM!"); 
}