#include "UART.h"

//==================================================UART0=================================================================

Motor update_motors[NUM_MOTORS];
SemaphoreHandle_t uart_mutex = NULL;

static const char *TAG = "UART_PID";

// === UART TX Queue Configuration ===
#define UART0_TX_QUEUE_LENGTH 50
#define UART0_TX_MSG_MAX_LEN 256

static QueueHandle_t uart0_tx_queue = NULL;


// UART initialization
void uart0_init(void)
{
    const uart_config_t uart0_config = {
        .baud_rate = UART0_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART0_PORT_NUM, &uart0_config));
    ESP_ERROR_CHECK(uart_set_pin(UART0_PORT_NUM, UART0_TX_PIN, UART0_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART0_PORT_NUM, UART0_BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART initialized on port %d at %d baud", UART0_PORT_NUM, UART0_BAUD_RATE);
}


// UART TX Queue Sender (Non-blocking)

void uart0_send_data(const char *data)
{
    if (uart0_tx_queue == NULL) return;

    size_t len = strlen(data);
    if (len >= UART0_TX_MSG_MAX_LEN) len = UART0_TX_MSG_MAX_LEN - 1;

    // Create a temporary message buffer
    char msg[UART0_TX_MSG_MAX_LEN];
    strncpy(msg, data, len);
    msg[len] = '\0';

    // Send to queue (with timeout)
    if (xQueueSend(uart0_tx_queue, msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "UART TX queue full, dropping message");
    }
}


// UART TX Task (single writer)

static void uart0_tx_task(void *arg)
{
    char tx_buffer[UART0_TX_MSG_MAX_LEN];

    while (1) {
        // Wait indefinitely for a message to send
        if (xQueueReceive(uart0_tx_queue, tx_buffer, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART0_PORT_NUM, tx_buffer, strlen(tx_buffer));
        }
        // small yield delay
        vTaskDelay(1);
    }
}


// UART RX Task (unchanged)

void uart0_rx_task(void *arg)
{
    uint8_t data[64];
    while (1) {
        int len = uart_read_bytes(UART0_PORT_NUM, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
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
                    uart0_send_data(msg);
                    uart0_send_data("OK\n");

                    ESP_LOGI(TAG, "Updated PID -> Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
                }
            }
        }
    }
}


//====================================================UART1==========================================================================


TaskHandle_t uart_send_task = NULL;
static esp_timer_handle_t uart_timer_handle;
const int uart_buffer_size = (256 * 2);

static QueueHandle_t uart_event_queue = NULL;

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install driver with RX/TX DMA buffers
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 20, &uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized with DMA, %d bps", UART_BAUD_RATE);
}

void IRAM_ATTR uart_timer_callback(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(uart_send_task, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void uart_timer_setup(void)
{
    esp_timer_create_args_t timer_args = {
        .callback = uart_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "uart_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &uart_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(uart_timer_handle, UART_UPDATE_RATE_US));
}

//=========UART Event Task (handles RX + async TX events)================

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[NUM_MOTORS * sizeof(float)];

    while (1) {
        // Wait for UART event from ISR
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {

                case UART_DATA: {
                    int len = uart_read_bytes(UART_PORT_NUM, data, event.size, pdMS_TO_TICKS(10));
                    if (len == NUM_MOTORS * sizeof(float)) {
                        union {
                            float f[NUM_MOTORS];
                            uint8_t b[NUM_MOTORS * 4];
                        } rx_packet;
                        memcpy(rx_packet.b, data, len);
                        for (int i = 0; i < NUM_MOTORS; i++) {
                            ESP_LOGI(TAG, "RX[%d] = %.3f", i, rx_packet.f[i]);
                        }
                    }
                    break;
                }

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "HW FIFO overflow");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring buffer full");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                default:
                    break;
            }
        }
    }
}


// UART Application Entry
void UART_app_main(void){

 //===============================UART0=============================================
    uart0_init();

    // Create TX queue
    uart0_tx_queue = xQueueCreate(UART0_TX_QUEUE_LENGTH, UART0_TX_MSG_MAX_LEN);
    if (uart0_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create UART TX queue");
        return;
    }

    //Create UART mutex to prevent data corruption 
    uart_mutex = xSemaphoreCreateMutex(); 
    if (uart_mutex == NULL) { 
        ESP_LOGE(TAG, "Failed to create UART mutex"); 
    }

    // Create TX task (high priority, single writer)
    xTaskCreatePinnedToCore(uart0_tx_task, "uart_tx_task", 4096, NULL, 12, NULL, 0);

    // Create RX task
    static TaskHandle_t UART0_task_handle;
    xTaskCreatePinnedToCore(uart0_rx_task,
                            "uart_rx_task",
                            4096,
                            NULL,
                            7,
                            &UART0_task_handle,
                            1);

 //=====================================================UART1===================================================

 ESP_LOGI(TAG, "setting up UART " );

 ESP_LOGI(TAG, "setting up UART MASTER device" );
 
 uart_init();
 ESP_LOGI(TAG, "setting up UART MASTER device Complete");

 //uart_timer_setup();

}