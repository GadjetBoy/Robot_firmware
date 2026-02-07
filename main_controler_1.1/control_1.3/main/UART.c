#include "control_1.3.h"
#include "UART.h"

Motor update_motors[NUM_MOTORS];

SemaphoreHandle_t uart_mutex = NULL;

static const char *TAG = "UART_PID";

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART initialized on port %d at %d baud", UART_PORT_NUM, UART_BAUD_RATE);
}

void uart_send_data(const char *data){

  if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE){ 
     uart_write_bytes(UART_PORT_NUM, data, strlen(data));
     xSemaphoreGive(uart_mutex);
    }
}



void uart_rx_task(void *arg){
    uint8_t data[64];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // null-terminate
            ESP_LOGI(TAG, "Received: %s", data);

            if (strncmp((char*)data, "SET_PID", 7) == 0) {
                float kp, ki, kd;
                if (sscanf((char*)data, "SET_PID,%f,%f,%f", &kp, &ki, &kd) == 3) {
                    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE){
                     for (int i = 0; i < NUM_MOTORS; i++){
                         update_motors[i].pos_pid.Kp = kp;
                         update_motors[i].pos_pid.Ki = ki;
                         update_motors[i].pos_pid.Kd = kd;   
                        }
                     xSemaphoreGive(uart_mutex);
                    }

                    char msg[64];
                    snprintf(msg, sizeof(msg), "PID_UPDATED,%.3f,%.3f,%.3f\n", kp, ki, kd);
                    uart_send_data(msg);  // send back to PC

                    ESP_LOGI(TAG, "Updated PID -> Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
                    uart_send_data("OK\n");
                    
                }
            }
        }
    }
}

void UART_app_main(void){
    
  uart_init();

    //Create UART mutex to prevent data corruption
    uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create UART mutex");
    }
    
 xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
}