#include "control_1.8.h"
#include "pcnt.h"
#include "cpg.h"
#include "ble.h"
#include "uart.h"

#define LED_PIN          12

static const char *TAG = "UART MASTER";

void read_PID_commands(float *dest) {
    uint8_t read_buffer;
    
    taskENTER_CRITICAL(&pid_buffer_swap_mux);
    read_buffer = current_write_index;
    
    memcpy(dest,(void*) motor_cmd_buffer[read_buffer], NUM_MOTORS * sizeof(float));
    
    taskEXIT_CRITICAL(&pid_buffer_swap_mux);
}

void uart_send_task_func(void *param) {
    union {
        float pwm_data[NUM_MOTORS];
        uint8_t b[NUM_MOTORS * 4];
    } packet;
    
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        read_PID_commands(packet.pwm_data);
        
        // DEBUG: Check what we're sending
        //ESP_LOGI("UART_MASTER", "Sending PWM: [0]=%.2f [4]=%.2f",packet.pwm_data[0], packet.pwm_data[4]);
        
        uint8_t tx_packet[PACKET_SIZE];
        tx_packet[0] = PACKET_HEADER_1;
        tx_packet[1] = PACKET_HEADER_2;
        memcpy(&tx_packet[2], packet.b, NUM_MOTORS * sizeof(float));
        
        int written = uart_write_bytes(UART_PORT_NUM, (const char*)tx_packet, PACKET_SIZE);
        if (written != PACKET_SIZE) {
            ESP_LOGE("UART_MASTER", "Write failed: %d/%d bytes", written, PACKET_SIZE);
        }
        
        esp_err_t res = uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(10));
        if (res != ESP_OK) {
            ESP_LOGE("UART_MASTER", "TX timeout");
        }
    }
}

void app_main(void){

 ESP_LOGI(TAG, "decorder initialization begins");
 
 gpio_reset_pin(LED_PIN);
 gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); 
 gpio_set_level(LED_PIN, 1);
 vTaskDelay(pdMS_TO_TICKS(500));
 gpio_set_level(LED_PIN, 0);

 xTaskCreatePinnedToCore(uart_send_task_func, 
                            "uart_send_task", 
                             5119, 
                             NULL, 
                             20, 
                             &uart_send_task, 
                             0);

 PCNT_app_main();
 vTaskDelay(pdMS_TO_TICKS(10));
 UART_app_main();
 vTaskDelay(pdMS_TO_TICKS(10));
 pid_app_main();
 vTaskDelay(pdMS_TO_TICKS(2));
 BLE_app_main();
 vTaskDelay(pdMS_TO_TICKS(10));
 CPG_app_main();
 vTaskDelay(pdMS_TO_TICKS(10));
 

 //esp_log_level_set("*", ESP_LOG_ERROR);

 esp_reset_reason_t reason = esp_reset_reason();
 ESP_LOGE("RESET", "CPU Reset reason: %d", reason);

 ESP_LOGI(TAG, "decorder initialization complete");
 
}


