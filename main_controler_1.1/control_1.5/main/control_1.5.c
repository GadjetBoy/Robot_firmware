#include "control_1.5.h"
#include "PID.h"
#include "PWM.h"
#include "CPG.h"
#include "BLE.h"
#include "UART.h"

#define I2C_PORT                 I2C_NUM_0  
#define I2C_SLAVE_SCL_IO         15
#define I2C_SLAVE_SDA_IO         13
#define SLAVE_ADDR               0x77
#define DATA_LENGTH              (sizeof(encoder_packet_t)*2)

#define TAG "I2C SLAVE"
#define QUEUE_LENGTH             20
#define GPIO_NUM                 12

#define i2c_log_freq             100

// ========================================= STRUCT DEFINITIONS ========================================= //

typedef struct {
    encoder_packet_t packet;
    size_t len;
} i2c_message_t;

// ========================================= GLOBAL VARIABLES ========================================= //

i2c_slave_context_t context = {0};
volatile int32_t encorder_val[NUM_MOTORS] = {0};

QueueHandle_t encoder_data_queue;
QueueHandle_t log_queue;

TaskHandle_t Print_task = NULL;
TaskHandle_t blinkTaskHandle = NULL;

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

    if (xQueueOverwriteFromISR(context->recieve_queue, &rx_msg.packet, &xHigherPriorityTaskWoken) != pdTRUE) {
        context->error = true;
    }

    vTaskNotifyGiveFromISR(pid_loop_task, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(blinkTaskHandle, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }

    return (xHigherPriorityTaskWoken == pdTRUE);
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

    context.recieve_queue = xQueueCreate(1, sizeof(encoder_packet_t));
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

    xTaskCreatePinnedToCore(PCNT_Print_Task, "Print_task", 3072, NULL, 4, &Print_task, 1);
    xTaskCreatePinnedToCore(blinkLed, "blink_task", 1024, NULL, 5, &blinkTaskHandle, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Setting up PWM and PID module");
    
    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGE("RESET", "CPU Reset reason: %d", reason);

    pwm_app_main();
    vTaskDelay(pdMS_TO_TICKS(10));
    UART_app_main();
    vTaskDelay(pdMS_TO_TICKS(10));
    pid_app_main();
    I2C_Salve_Device_Setup(&context);
    ESP_LOGI(TAG, "Slave initialization complete and ready to receive data");
    vTaskDelay(pdMS_TO_TICKS(10));
    mot_command_app_main();
    BLE_app_main();
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_log_level_set("*", ESP_LOG_ERROR);
   
}
