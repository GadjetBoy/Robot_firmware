#include "control_1.2.h"
#include "PID.h"
#include "PWM.h"
#include "motor_command.h"
#include "BLE.h"
#include "UART.h"

#define I2C_PORT                 I2C_NUM_0  
#define I2C_SLAVE_SCL_IO         15
#define I2C_SLAVE_SDA_IO         13
#define SLAVE_ADDR               0x77

#define TAG "I2C SLAVE"
#define QUEUE_LENGTH             15
#define GPIO_NUM                 12

#define i2c_log_freq             100

#define NUM_MOTORS               8
#define DATA_LENGTH              (sizeof(encoder_packet_t))

// ========================================= STRUCT DEFINITIONS ========================================= //

typedef struct __attribute__((packed)) {
    int32_t encoders[NUM_MOTORS];
} encoder_packet_t;

typedef struct {
    encoder_packet_t packet;
    size_t len;
} i2c_message_t;

typedef struct {
    i2c_slave_dev_handle_t slave_handle;
    QueueHandle_t recieve_queue;
    bool error;
} i2c_slave_context_t;

// ========================================= GLOBAL VARIABLES ========================================= //

static i2c_slave_context_t context = {0};
volatile int32_t encorder_val[NUM_MOTORS] = {0};

QueueHandle_t log_queue;

TaskHandle_t Recieve_task = NULL;
TaskHandle_t Print_task = NULL;
TaskHandle_t blinkTaskHandle = NULL;

portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// ========================================= FUNCTION DECLARATIONS ========================================= //

void I2C_Salve_Device_Setup(i2c_slave_context_t *context);
static bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel,
                                       const i2c_slave_rx_done_event_data_t *evt_data,
                                       void *user_data);
void Data_Recive_task(void *arg);
void PCNT_Print_Task(void *Parameters);
void blinkLed(void *pvValue);

// ========================================= I2C SLAVE SETUP ========================================= //

void I2C_Salve_Device_Setup(i2c_slave_context_t *context) {
    i2c_slave_config_t i2c_slv_config = {
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .send_buf_depth = DATA_LENGTH,
        .receive_buf_depth = DATA_LENGTH,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .slave_addr = SLAVE_ADDR,
        .intr_priority = 3,
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &(context->slave_handle)));

    i2c_slave_event_callbacks_t Call_back = {
        .on_receive = i2c_slave_rx_done_callback,
    };

    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(context->slave_handle, &Call_back, context));
}

// ========================================= CALLBACK ========================================= //

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel,
                                                 const i2c_slave_rx_done_event_data_t *evt_data,
                                                 void *user_data) {
    i2c_slave_context_t *context = (i2c_slave_context_t *)user_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    i2c_message_t rx_msg;
    memcpy(&rx_msg.packet, evt_data->buffer, sizeof(encoder_packet_t));
    rx_msg.len = DATA_LENGTH;

    if (xQueueSendFromISR(context->recieve_queue, &rx_msg, &xHigherPriorityTaskWoken) != pdTRUE) {
        context->error = true;
    }

    vTaskNotifyGiveFromISR(Recieve_task, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(blinkTaskHandle, &xHigherPriorityTaskWoken);

    return (xHigherPriorityTaskWoken == pdTRUE);
}

// ========================================= DATA RECEIVE TASK ========================================= //

void IRAM_ATTR Data_Recive_task(void *arg) {
    i2c_slave_context_t *context = (i2c_slave_context_t *)arg;
    i2c_message_t msg;
    static uint8_t log_counter = 0;

    while (1) {
        uint32_t notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        log_counter++;

        if (log_counter >= i2c_log_freq) {
            ESP_LOGI(TAG, "Receiving ..... (notify count: %lu)\n", notify_count);
        }

        for (uint32_t i = 0; i < notify_count; i++) {
            if (xQueueReceive(context->recieve_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (context->error) {
                    ESP_LOGE(TAG, "RX queue overflow detected earlier\n");
                    context->error = false;
                }

                taskENTER_CRITICAL(&encoder_mux);
                memcpy((void *)encorder_val, msg.packet.encoders, sizeof(msg.packet.encoders));
                taskEXIT_CRITICAL(&encoder_mux);

                if (log_counter >= i2c_log_freq) {
                    ESP_LOGI(TAG, "Data Received - Encoder values updated\n");
                    log_counter = 0;
                }

                xQueueOverwrite(log_queue, &msg);
            } else {
                ESP_LOGE(TAG, "Queue receive failed (possible mismatch or timeout)\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ========================================= PRINT TASK ========================================= //

void PCNT_Print_Task(void *Parameters) {
    i2c_message_t msg;

    while (1) {
        if (xQueueReceive(log_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "\n"
              "ENCODER[1] READING: %" PRId32 "\n"
              "ENCODER[2] READING: %" PRId32 "\n"
              "ENCODER[3] READING: %" PRId32 "\n"
              "ENCODER[4] READING: %" PRId32 "\n"
              "ENCODER[5] READING: %" PRId32 "\n"
              "ENCODER[6] READING: %" PRId32 "\n"
              "ENCODER[7] READING: %" PRId32 "\n"
              "ENCODER[8] READING: %" PRId32,
              msg.packet.encoders[0], msg.packet.encoders[1],
              msg.packet.encoders[2], msg.packet.encoders[3],
              msg.packet.encoders[4], msg.packet.encoders[5],
              msg.packet.encoders[6], msg.packet.encoders[7]);

            ESP_LOGI(TAG, ".............\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
    gpio_reset_pin(GPIO_NUM);
    gpio_set_direction(GPIO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM, 0);

    context.recieve_queue = xQueueCreate(QUEUE_LENGTH, sizeof(i2c_message_t));
    if (!context.recieve_queue) {
        ESP_LOGE(TAG, "Creating receive queue failed");
        return;
    }

    log_queue = xQueueCreate(1, sizeof(i2c_message_t));
    if (!log_queue) {
        ESP_LOGE(TAG, "Creating log queue failed");
        return;
    }

    context.error = false;

    I2C_Salve_Device_Setup(&context);
    ESP_LOGI(TAG, "Slave initialization complete and ready to receive data");

    xTaskCreatePinnedToCore(Data_Recive_task, "I2C_receive_Task", 5120, &context, 10, &Recieve_task, 0);
    xTaskCreatePinnedToCore(PCNT_Print_Task, "Print_task", 3072, NULL, 5, &Print_task, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(blinkLed, "blink_task", 1024, NULL, 6, &blinkTaskHandle, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Setting up PWM and PID module");

    // Disable ESP logging to prevent UART conflicts
    esp_log_level_set("*",ESP_LOG_WARN);
    
    
    pwm_app_main();
    UART_app_main();
    pid_app_main();
    mot_command_app_main();
    BLE_app_main();
    
}
