#include "control_1.7.h"
#include "pwm.h"
#include "set_motor.h"
#define UART_PORT_NUM UART_NUM_1
#define TXD_PIN 13
#define RXD_PIN 15
#define UART_BAUD_RATE 2000000
// DMA buffer size (must be multiple of 4 and >= packet size)
#define UART_DMA_RX_SIZE 4096 // Increased for bursts
#define UART_FIFO_LENGTH 256
static const char *TAG = "UART_SLAVE_DMA";
#define QUEUE_LENGTH 20
#define GPIO_NUM 12
#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55
#define PACKET_SIZE (2 + NUM_MOTORS * sizeof(float)) // 34 bytes
#define RX_BUFFER_SIZE 2048 // Doubled for headroom
// ========================================= GLOBAL VARIABLES ========================================= //
QueueHandle_t encoder_data_queue;
QueueHandle_t log_queue;
static QueueHandle_t uart_event_queue = NULL;
TaskHandle_t Print_task = NULL;
TaskHandle_t blinkTaskHandle = NULL;
volatile DRAM_ATTR float motor_cmd_buffer[2][NUM_MOTORS];
volatile DRAM_ATTR uint8_t current_write_index = 0; // PID reads from this
portMUX_TYPE pid_buffer_swap_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_head = 0;
uint16_t rx_tail = 0;
// ========================================= FUNCTION DECLARATIONS ========================================= //
void uart_init(void);
void uart_receive_task(void *param);
void PCNT_Print_Task(void *Parameters);
void blinkLed(void *pvValue);
void update_UART_motor_commands(float *new_commands);
extern void pid_app_main();
// ========================================= UART INIT ========================================= //
void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Change to UART_HW_FLOWCTRL_CTS_RTS if master supports
        .source_clk = UART_SCLK_APB,
        .rx_flow_ctrl_thresh = 120U // Safe value <=255 to avoid overflow warning
    };
    // Install with no TX buffer, larger event queue
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_DMA_RX_SIZE, 0, 40, &uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Lower threshold for more frequent drains
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_PORT_NUM, UART_FIFO_LENGTH / 8)); // ~32
    ESP_LOGI(TAG, "UART initialized with DMA, %d bps", UART_BAUD_RATE);
}
void uart_receive_task(void *param){
    // Use DMA-capable memory for receiving (static/global for stack safety, aligned for DMA)
    static uint8_t dma_rx[UART_DMA_RX_SIZE] __attribute__((aligned(4))) = {0};
    static uint8_t temp_packet[PACKET_SIZE] __attribute__((aligned(4))); // Temp for linear copy (header + data)
    uart_event_t event;
    static int drop_counter = 0; // Throttle drop logs
    static int event_count = 0; // For debug
    static int parse_log_counter = 0; // Throttle parse logs
    while (1) {
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    int len = 0;
                    // Fix: Read event.size first with small timeout to capture triggered data
                    int initial = uart_read_bytes(UART_PORT_NUM, dma_rx, event.size, pdMS_TO_TICKS(1));
                    if (initial > 0) len = initial;
                    int total_read = initial;
                    // Then drain extras non-blockingly
                    do {
                        int chunk = uart_read_bytes(UART_PORT_NUM, dma_rx + total_read,
                                                    UART_DMA_RX_SIZE - total_read, 0);
                        if (chunk <= 0) break;
                        total_read += chunk;
                        len += chunk;
                    } while (total_read < UART_DMA_RX_SIZE);
                  
                    if (len > 0) {
                        // Copy with overflow: Drop aggressively if < space for 2 packets
                        int buf_used = (rx_head + RX_BUFFER_SIZE - rx_tail) % RX_BUFFER_SIZE;
                        int buf_avail = RX_BUFFER_SIZE - buf_used;
                        if (buf_avail < PACKET_SIZE * 2) {
                            int to_drop = (PACKET_SIZE * 2 - buf_avail);
                            rx_tail = (rx_tail + to_drop) % RX_BUFFER_SIZE;
                            if (++drop_counter % 10 == 0) {
                                ESP_LOGW(TAG, "Pre-copy drop: %d bytes (events: %d)", to_drop, drop_counter);
                            }
                        }
                        // Bulk copy to circular buffer
                        for (int i = 0; i < len; i++) {
                            rx_buffer[rx_head] = dma_rx[i];
                            rx_head = (rx_head + 1) % RX_BUFFER_SIZE;
                        }
                      
                        // Debug log every 200 events (reduced overhead)
                       /* buf_used = (rx_head + RX_BUFFER_SIZE - rx_tail) % RX_BUFFER_SIZE;
                        if (++event_count % 200 == 0) {
                            ESP_LOGI(TAG, "Event #%d: Rx %d bytes, buf used %d/%d (%.1f%%)", event_count, len, buf_used, RX_BUFFER_SIZE, (float)buf_used / RX_BUFFER_SIZE * 100);
                        }*/
                      
                        // Optimized batch parsing (up to 10 packets/event)
                        int packets_parsed = 0;
                        size_t avail = (rx_head + RX_BUFFER_SIZE - rx_tail) % RX_BUFFER_SIZE;
                        int max_packets = (avail / PACKET_SIZE > 10) ? 10 : (avail / PACKET_SIZE);
                        for (int pkt = 0; pkt < max_packets; pkt++) {
                            // Localize tail for fewer modulos
                            size_t cur_tail = rx_tail;
                            uint8_t *tail_ptr = &rx_buffer[cur_tail % RX_BUFFER_SIZE];
                            // Fast header check (pointer + offset)
                            if (tail_ptr[0] == PACKET_HEADER_1 && tail_ptr[1] == PACKET_HEADER_2) {
                                // Copy full packet to temp linear buffer (handle wrap)
                                size_t bytes_to_end = RX_BUFFER_SIZE - (cur_tail % RX_BUFFER_SIZE);
                                if (bytes_to_end >= PACKET_SIZE) {
                                    memcpy(temp_packet, &rx_buffer[cur_tail % RX_BUFFER_SIZE], PACKET_SIZE);
                                } else {
                                    // Wrap: copy to end + from start
                                    memcpy(temp_packet, &rx_buffer[cur_tail % RX_BUFFER_SIZE], bytes_to_end);
                                    memcpy(temp_packet + bytes_to_end, rx_buffer, PACKET_SIZE - bytes_to_end);
                                }
                                // Extract floats from linear temp (no loops/modulos)
                                float packet_data[NUM_MOTORS];
                                memcpy(packet_data, temp_packet + 2, NUM_MOTORS * sizeof(float)); // Direct 32-byte copy
                                // Update
                                update_UART_motor_commands(packet_data);
                                // Log throttled parse success (for debugging)
                               /* if (++parse_log_counter % 50 == 0) {
                                    ESP_LOGI(TAG, "Parsed %d pkts total (latest motor0=%.1f)", parse_log_counter, packet_data[0]);
                                }*/
                                // Advance tail
                                rx_tail = (rx_tail + PACKET_SIZE) % RX_BUFFER_SIZE;
                                packets_parsed++;
                                // Notify every packet (more frequent for loop)
                                xTaskNotifyGive(pid_loop_task);
                                xTaskNotifyGive(blinkTaskHandle);
                            } else {
                                // Fast header resync: scan forward up to 5 bytes
                                bool found = false;
                                int skips_tried = 0;
                                for (int skip = 1; skip <= 5; skip++) {
                                    skips_tried = skip;
                                    size_t next_tail = (cur_tail + skip) % RX_BUFFER_SIZE;
                                    uint8_t *next_ptr = &rx_buffer[next_tail % RX_BUFFER_SIZE];
                                    if (next_ptr[0] == PACKET_HEADER_1 && next_ptr[1] == PACKET_HEADER_2) {
                                        rx_tail = next_tail;
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found) {
                                    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE; // Nibble
                                }
                                // Early exit if resync fails often (backpressure hint)
                                if (skips_tried > 3) break;
                            }
                        }
                    } else if (event.size > 0) {
                        // Only warn if expected data but none read (now rarer)
                        ESP_LOGD(TAG, "Transient event: expected %d bytes but read 0", event.size); // DEBUG to reduce noise
                    }
                    break;
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_event_queue);
                    ESP_LOGW(TAG, "%s - flushed & reset", event.type == UART_FIFO_OVF ? "HW FIFO overflow" : "Ring buffer full");
                    rx_head = rx_tail = 0;
                    drop_counter = 0;
                    event_count = 0;
                    parse_log_counter = 0;
                    break;
                default:
                    break;
            }
        }
    }
}
void update_UART_motor_commands(float *new_commands) {
    uint8_t write_buffer = 1 - current_write_index; // Write to opposite buffer
  
    // Copy to back buffer
    memcpy((void*)motor_cmd_buffer[write_buffer], new_commands, NUM_MOTORS * sizeof(float));
    // Atomic swap - ensures PID sees consistent snapshot
    taskENTER_CRITICAL(&pid_buffer_swap_mux);
    current_write_index = write_buffer;
    taskEXIT_CRITICAL(&pid_buffer_swap_mux);
}
// ========================================= PRINT TASK ========================================= //
void PCNT_Print_Task(void *Parameters) {
    pwm_packet_t msg;
    while (1) {
        if (xQueueReceive(log_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "\n"
              "PWM[1] READING: %.2f \n"
              "PWM[2] READING: %.2f \n"
              "PWM[3] READING: %.2f \n"
              "PWM[4] READING: %.2f \n"
              "PWM[5] READING: %.2f \n"
              "PWM[6] READING: %.2f \n"
              "PWM[7] READING: %.2f \n"
              "PWM[8] READING: %.2f" ,
              msg.target_pwm_val[0], msg.target_pwm_val[1],
              msg.target_pwm_val[2], msg.target_pwm_val[3],
              msg.target_pwm_val[4], msg.target_pwm_val[5],
              msg.target_pwm_val[6], msg.target_pwm_val[7]);
            ESP_LOGI(TAG, ".............\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
// ========================================= LED BLINK TASK ========================================= //
void blinkLed(void *pvValue) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gpio_set_level(GPIO_NUM, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
// ========================================= APP MAIN ========================================= //
void app_main(void) {
    // Explicitly zero the buffer
    memset((void*)motor_cmd_buffer, 0, sizeof(motor_cmd_buffer));
  
    gpio_reset_pin(GPIO_NUM);
    gpio_set_direction(GPIO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM, 0);
    log_queue = xQueueCreate(1, sizeof(pwm_packet_t)); // Fixed: Larger queue to unblock prints
    if (!log_queue) {
        ESP_LOGE(TAG, "Creating log queue failed");
        return;
    }
  
    // Increase WDT timeout to 10s as interim fix
    //esp_task_wdt_init(10, true); // 10s timeout, panic on trigger
  
    uart_init();
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Setting up PWM and PID module");
    pwm_app_main();
    vTaskDelay(pdMS_TO_TICKS(10));
    set_motor_app_main();
    // Note: pid_app_main() removed as it may not be defined; add back if available
  
    // Suppress noisy logs completely
    esp_log_level_set("MOTOR_SET", ESP_LOG_NONE); // Disables all MOTOR_SET output
  
    xTaskCreatePinnedToCore(blinkLed, "blink_task", 1024, NULL, 5, &blinkTaskHandle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(uart_receive_task, "uart_receive_task", 8192, NULL, 12, NULL, 0); // Core 0, higher prio, more stack
    xTaskCreatePinnedToCore(PCNT_Print_Task, "Print_task", 3072, NULL, 4, &Print_task, 1);
  
    ESP_LOGI(TAG, "Slave initialization complete and ready to receive data");
    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGE("RESET", "CPU Reset reason: %d", reason);
}