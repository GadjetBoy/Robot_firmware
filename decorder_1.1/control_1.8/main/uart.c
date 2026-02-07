#include "uart.h"

#define UART_TAG "UART"

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

    ESP_LOGI(UART_TAG, "UART initialized with DMA, %d bps", UART_BAUD_RATE);
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
                            ESP_LOGI(UART_TAG, "RX[%d] = %.3f", i, rx_packet.f[i]);
                        }
                    }
                    break;
                }

                case UART_FIFO_OVF:
                    ESP_LOGW(UART_TAG, "HW FIFO overflow");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(UART_TAG, "Ring buffer full");
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

 ESP_LOGI(UART_TAG, "setting up UART " );

 ESP_LOGI(UART_TAG, "setting up UART MASTER device" );
 
 uart_init();
 
 ESP_LOGI(UART_TAG, "setting up UART MASTER device Complete");

 //uart_timer_setup();

}