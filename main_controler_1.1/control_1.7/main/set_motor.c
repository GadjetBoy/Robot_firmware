#include "set_motor.h"

#define TAG "MOTOR_SET"

// ====================== Globals ======================
motor_Command_t motors[NUM_MOTORS];
QueueHandle_t motor_command_queue;
TaskHandle_t pid_loop_task = NULL;
esp_timer_handle_t motor_timer_handle = NULL;

// Forward declarations
void drive_motor(motor_Command_t* m);
void stop_motor(motor_Command_t *m);

// ====================== Motor Control ======================
void drive_motor(motor_Command_t* m) {

     esp_err_t ret1, ret2;

    if (m->direction == -1) {
        ret1 =ledc_set_duty(LEDC_MODEA, m->pwm_channelA, 0);
        ret2 =ledc_set_duty(LEDC_MODEB, m->pwm_channelB, m->pwm);
    
    } else {
        ret1 =ledc_set_duty(LEDC_MODEA, m->pwm_channelA, m->pwm);
        ret2 =ledc_set_duty(LEDC_MODEB, m->pwm_channelB, 0);
    }

    //log errors if they occur (outside of timing-critical section)
    if (ret1 != ESP_OK || ret2 != ESP_OK) {
        // Use printf instead of ESP_LOG to avoid lock issues
        printf("LEDC set_duty error: chA=%d, chB=%d\n", ret1, ret2);
    }

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, m->pwm_channelA));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, m->pwm_channelB));
   
}

void stop_motor(motor_Command_t *m) {
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, m->pwm_channelA, PWM_STOP));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, m->pwm_channelB, PWM_STOP));
    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, m->pwm_channelA));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, m->pwm_channelB));
}
static inline void apply_break(motor_Command_t *m){
 
 ESP_ERROR_CHECK(ledc_stop(LEDC_MODEA, m->pwm_channelA, 1)); // Set idle level to HIGH
 ESP_ERROR_CHECK(ledc_stop(LEDC_MODEB, m->pwm_channelB, 1)); // Set idle level to HIGH
 
}

void read_PID_commands(float *dest) {
    uint8_t read_buffer;
    
    taskENTER_CRITICAL(&pid_buffer_swap_mux);
    read_buffer = current_write_index;
    
    
    memcpy(dest,(void*) motor_cmd_buffer[read_buffer], sizeof(float) * NUM_MOTORS);
    
    taskEXIT_CRITICAL(&pid_buffer_swap_mux);
}

// ====================== Position Loop ======================
void position_loop_task(void *arg) {
    static float current_pwm[NUM_MOTORS] = {0};
    static pwm_packet_t pwm_packet;  // Full packet for queue
    static int log_counter = 0;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        read_PID_commands(current_pwm);

        // Apply to all motors
        for (int i = 0; i < NUM_MOTORS; i++) {

            float pid_output = current_pwm[i];

            int new_dir = (pid_output > 0) ? 1 : -1;

            if (motors[i].direction != new_dir) {
                motors[i].direction = new_dir;
            }

            float PID = fabsf(pid_output);
            uint32_t pwm_magnitude = (uint32_t)PID; 

            if(pwm_magnitude <= MIN_DUTY && pwm_magnitude > 0){
               pwm_magnitude = MIN_DUTY;
            }

            motors[i].pwm = pwm_magnitude;
            motors[i].last_pwm = pwm_magnitude;  // Keep for logging if needed


            drive_motor(&motors[i]);
            // Pack into packet
            pwm_packet.target_pwm_val[i] = (float)motors[i].pwm * (pid_output > 0 ? 1.0f : -1.0f);  // Preserve sign
        }
        // Send full packet ONCE after loop
        /*if (xQueueOverwrite(log_queue, &pwm_packet) != pdTRUE) {
            ESP_LOGE(TAG, "Log queue overwrite failed");
        }*/
        // Throttled log (every 100 loops ~50ms at 2kHz)
        /*if (++log_counter % 200 == 0) {
          float pid_out_log = pwm_packet.target_pwm_val[4];
          ESP_LOGE(TAG, "Applied to motor[4]: pwm=%.0f dir=%d",pid_out_log,motors[4].direction);
        }*/

    }
}
static void IRAM_ATTR motor_timer_callback(void *arg) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(pid_loop_task, &higher_priority_task_woken);
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void Set_motorLoop_timer(void){

// Setup timer for periodic wakeup
    esp_timer_create_args_t timer_args = {
        .callback = &motor_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,  // Or TASK for non-ISR if needed
        .name = "motor_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &motor_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(motor_timer_handle, MOTOR_UPDATE_RATE_US));
    ESP_LOGI(TAG, "Motor timer started at %.2f Hz", 1000000 / MOTOR_UPDATE_RATE_US);
}

// ====================== Motor Setup ======================
void set_motor_app_main(void) {
    ESP_LOGI(TAG, "Starting PID application...");

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i + 1;
        motors[i].pwm_channelA = LEDC_A_CHANNEL[i];
        motors[i].pwm_channelB = LEDC_B_CHANNEL[i];
    }

    xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 5120, NULL, 20, &pid_loop_task,0);

    vTaskDelay(pdMS_TO_TICKS(100));

    Set_motorLoop_timer();

    ESP_LOGI(TAG, "PID application started successfully");
}  