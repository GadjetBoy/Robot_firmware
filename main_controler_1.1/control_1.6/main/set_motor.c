#include "set_motor.h"

// ====================== Globals ======================
QueueHandle_t motor_command_queue;
TaskHandle_t pid_loop_task = NULL;

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

// ====================== Position Loop ======================
void position_loop_task(void *arg) {
    
    encoder_packet_t current_pwm = {0};  // This packet will hold our local, consistent copy of all encoder value
    static uint32_t i2c_error_count = 0;

    while (1) {

        // At the start of the loop, get the MOST RECENT encoder snapshot
        if (xQueueReceive(context.recieve_queue, &current_pwm,portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "i2c pwm data received\n");
        }
        else {
            // Use existing data, don't log error every time
            i2c_error_count++;
            if (i2c_error_count % 100 == 0) {  // Only log occasionally
                ESP_LOGW(TAG, "Receiving pwm values failed:No new data, using previous (count: %lu)", i2c_error_count);
                i2c_error_count=0;
            }
        }

        for (int i = 0; i < NUM_MOTORS; i++) {

            float pid_output = current_pwm.target_pwm_val[i];
            //ESP_LOGE(TAG,"PWM[%1d] READING: %.2f \n",i,pid_output);

            int new_dir = (pid_output > 0) ? 1 : -1;
            if (motors[i].direction != new_dir){
              motors[i].direction = new_dir;
            }
            
            float PID = fabsf(pid_output);
            uint32_t pwm_magnitude = (uint32_t)PID;

            if(pwm_magnitude < MIN_DUTY){
              pwm_magnitude = MIN_DUTY;
            }

            motors[i].pwm = pwm_magnitude;
            motors[i].last_pwm = pwm_magnitude;

            if (motors[i].pwm == 0) {
                 stop_motor(&motors[i]);
                 apply_break(&motors[i]); // Apply brake when idle
                 continue; 
            }

            drive_motor(&motors[i]);

            xQueueOverwrite(log_queue,&motors[i].last_pwm);
        }
    }
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

    ESP_LOGI(TAG, "PID application started successfully");
}  