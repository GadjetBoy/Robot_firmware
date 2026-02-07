#include "UART.h"

Motor update_motors[NUM_MOTORS];
SemaphoreHandle_t uart_mutex = NULL;

static const char *TAG = "UART_PID";

// === UART TX Queue Configuration ===
#define UART_TX_QUEUE_LENGTH 50
#define UART_TX_MSG_MAX_LEN 256

static QueueHandle_t uart_tx_queue = NULL;


// UART initialization
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


// UART TX Queue Sender (Non-blocking)

void uart_send_data(const char *data)
{
    if (uart_tx_queue == NULL) return;

    size_t len = strlen(data);
    if (len >= UART_TX_MSG_MAX_LEN) len = UART_TX_MSG_MAX_LEN - 1;

    // Create a temporary message buffer
    char msg[UART_TX_MSG_MAX_LEN];
    strncpy(msg, data, len);
    msg[len] = '\0';

    // Send to queue (with timeout)
    if (xQueueSend(uart_tx_queue, msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "UART TX queue full, dropping message");
    }
}


// UART TX Task (single writer)

static void uart_tx_task(void *arg)
{
    char tx_buffer[UART_TX_MSG_MAX_LEN];

    while (1) {
        // Wait indefinitely for a message to send
        if (xQueueReceive(uart_tx_queue, tx_buffer, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART_PORT_NUM, tx_buffer, strlen(tx_buffer));
        }
        // small yield delay
        vTaskDelay(1);
    }
}


// UART RX Task (unchanged)

void uart_rx_task(void *arg)
{
    uint8_t data[64];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // null-terminate
            ESP_LOGI(TAG, "Received: %s", data);

            if (strncmp((char*)data, "SET_PID", 7) == 0) {
                float kp, ki, kd;
                if (sscanf((char*)data, "SET_PID,%f,%f,%f", &kp, &ki, &kd) == 3) {
                    for (int i = 0; i < NUM_MOTORS; i++) {
                        update_motors[i].pos_pid.Kp = (float)kp;
                        update_motors[i].pos_pid.Ki = (float)ki;
                        update_motors[i].pos_pid.Kd = (float)kd;
                    }

                    char msg[64];
                    snprintf(msg, sizeof(msg), "PID_UPDATED,%.3f,%.3f,%.3f\n", kp, ki, kd);
                    uart_send_data(msg);
                    uart_send_data("OK\n");

                    ESP_LOGI(TAG, "Updated PID -> Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
                }
            }
        }
    }
}


// UART Application Entry
void UART_app_main(void)
{
    uart_init();

    // Create TX queue
    uart_tx_queue = xQueueCreate(UART_TX_QUEUE_LENGTH, UART_TX_MSG_MAX_LEN);
    if (uart_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create UART TX queue");
        return;
    }

    //Create UART mutex to prevent data corruption 
    uart_mutex = xSemaphoreCreateMutex(); 
    if (uart_mutex == NULL) { 
        ESP_LOGE(TAG, "Failed to create UART mutex"); 
    }

    // Create TX task (high priority, single writer)
    xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 4096, NULL, 8, NULL, 1);

    // Create RX task
    static TaskHandle_t UART_task_handle;
    xTaskCreatePinnedToCore(uart_rx_task,
                            "uart_rx_task",
                            4096,
                            NULL,
                            7,
                            &UART_task_handle,
                            1);
}
