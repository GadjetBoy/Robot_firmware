#include "PID.h"
#include "CPG.h"
#include "UART.h"

#define TAG "PID"
#define PID_QUEUE_LENGTH 1

// ====================== Globals ======================
static Motor motors[NUM_MOTORS];
QueueHandle_t motor_command_queue;
TaskHandle_t pid_loop_task = NULL;

// Forward declarations
void drive_motor(Motor* m);
void stop_motor(Motor *m);

// ====================== PID Functions ======================
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

void update_PID_gain(void){
    if (xSemaphoreTake(uart_mutex,0) == pdTRUE){
    for(int i=0;i<NUM_MOTORS;i++){
     motors[i].pos_pid.Kp = update_motors[i].pos_pid.Kp;
     motors[i].pos_pid.Ki = update_motors[i].pos_pid.Ki;
     motors[i].pos_pid.Kd = update_motors[i].pos_pid.Kd;
    }
    xSemaphoreGive(uart_mutex);
    }
}

float PIDController_Update(PIDController *pid, float setpoint, int32_t measurement, uint8_t i) {
    
    pid->prevError = pid->error;
    pid->lastOutput = pid->out;
    pid->prevMeasurement = (float)measurement;
  
    pid->error = setpoint - (float)measurement;

    // Proportional term
    pid->proportional = pid->Kp * pid->error;
    //derivative term
    pid->differentiator = pid->Kd*(pid->error - pid->prevError)/pid->T;
    //integral term
    pid->integrator += pid->Ki *pid->error;

    // Clamp integrator
    if (pid->integrator > pid->limMaxInt){
        pid->integrator = pid->limMaxInt;
    }
    if (pid->integrator < pid->limMinInt){
        pid->integrator = pid->limMinInt;
    }

    if(fabs(pid->error)<=STOP_THRESHOLD*10){
      pid->integrator *= 0.9f; // decay
    }

    //calculate PID output 
    pid->out = pid->proportional +pid->differentiator+pid->integrator;
    //normalize output for stability
    pid->out = 0.7 * pid->out + 0.3 * pid->lastOutput;

    //clamp PID output 

    if (pid->out > pid->limMax){
        pid->out = pid->limMax;
    }
    if (pid->out < pid->limMin){
        pid->out = pid->limMin;
    }

    return pid->out;
}

// ====================== Motor Control ======================
void drive_motor(Motor* m) {

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

void stop_motor(Motor *m) {
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, m->pwm_channelA, PWM_STOP));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, m->pwm_channelB, PWM_STOP));
    
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, m->pwm_channelA));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, m->pwm_channelB));
}
static inline void apply_break(Motor *m){
 
 ESP_ERROR_CHECK(ledc_stop(LEDC_MODEA, m->pwm_channelA, 1)); // Set idle level to HIGH
 ESP_ERROR_CHECK(ledc_stop(LEDC_MODEB, m->pwm_channelB, 1)); // Set idle level to HIGH
 
}

// ====================== Motor Functions ======================
void motor_update_position(Motor *m, int counts) {
    m->current_position = (float)counts;
}

// ====================== Position Loop ======================
void position_loop_task(void *arg) {
    
    uint8_t stall_count[NUM_MOTORS] = {0};
    static motor_Command_t cpg[NUM_MOTORS];
    encoder_packet_t current_encoders = {0};  // This packet will hold our local, consistent copy of all encoder value

    static uint32_t cpg_error_count = 0;
    static uint32_t i2c_error_count = 0;

    while (1) {

        update_PID_gain();

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xQueueReceive(motor_command_queue,cpg,pdMS_TO_TICKS(50)) == pdTRUE) {
            for(int i =0 ; i<NUM_MOTORS;i++){
             motors[i].target_position = cpg[i].motor_target;
             motors[i].active = cpg[i].activate;
            }
        } else{
            // Use existing data, don't log error every time
            cpg_error_count++;
            if (cpg_error_count % 10 == 0) {  // Only log occasionally
                ESP_LOGE(TAG, "Receiving cpg values failed (count: %lu)", cpg_error_count);
                cpg_error_count=0;
            }
        }

        // At the start of the loop, get the MOST RECENT encoder snapshot
        if (xQueueReceive(context.recieve_queue, &current_encoders,0) == pdTRUE) {
            ESP_LOGE(TAG, "i2c data received\n");
        }
        else {
            // Use existing data, don't log error every time
            i2c_error_count++;
            if (i2c_error_count % 100 == 0) {  // Only log occasionally
                ESP_LOGE(TAG, "Receiving encoder values failed:No new encoder data, using previous (count: %lu)", i2c_error_count);
                i2c_error_count=0;
            }
        }

        for (int i = 0; i < NUM_MOTORS; i++) {

            motors[i].current_position = current_encoders.encoders[i];

            float error = motors[i].target_position - (float)motors[i].current_position;
            float abs_error = fabsf(error);

            float pid_output = PIDController_Update(&motors[i].pos_pid,
                                                    motors[i].target_position,
                                                    motors[i].current_position, i);

            int new_dir = (pid_output > 0) ? 1 : -1;
            if (motors[i].direction != new_dir){
                motors[i].direction = new_dir;
            }
            
            float PID = fabsf(pid_output);
            uint32_t pwm_magnitude = (uint32_t)PID;

            if(pwm_magnitude < MIN_DUTY){
                   pwm_magnitude = MIN_DUTY;
            }
           

            if (abs_error > TOL) {
                if (motors[i].current_position == motors[i].last_enc_val) {
                    stall_count[i]++;
                    if (stall_count[i] >= 10) {
                        pwm_magnitude = PWM_MAX;
                        if(i==7){
                         ESP_LOGE(TAG, "mor[%1d]stall:[%1d]\n",i,stall_count[i]);
                        }
                        stall_count[i] = 0;
                    }
                }
            }

            motors[i].pwm = pwm_magnitude;
            motors[i].last_pwm = pwm_magnitude;
            motors[i].last_enc_val = motors[i].current_position;

            if (!motors[i].active) {
                if(motors[i].current_position<=STOP_THRESHOLD*5){
                 stop_motor(&motors[i]);
                 apply_break(&motors[i]); // Apply brake when idle
                 continue; 
                }
            }

            drive_motor(&motors[i]);
        }

       //xQueueOverwrite(log_queue, &msg);
       //vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ====================== Motor Setup ======================
void pid_app_main(void) {
    ESP_LOGI(TAG, "Starting PID application...");
    
    motor_command_queue = xQueueCreate(PID_QUEUE_LENGTH, sizeof(motor_Command_t)*NUM_MOTORS);
    if (!motor_command_queue) {
        ESP_LOGE(TAG, "Creating motor command queue failed");
        return;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i + 1;
        motors[i].target_position = 0.0f;
        motors[i].current_position = 0;
        motors[i].pwm_channelA = LEDC_A_CHANNEL[i];
        motors[i].pwm_channelB = LEDC_B_CHANNEL[i];
        motors[i].active = false;

        motors[i].pos_pid.Kp = 0.00f;
        motors[i].pos_pid.Ki = 0.00f;
        motors[i].pos_pid.Kd = 0.00f;

        motors[i].pos_pid.T = POS_DT;
        motors[i].pos_pid.limMin = -PWM_MAX;
        motors[i].pos_pid.limMax = PWM_MAX;
        motors[i].pos_pid.limMinInt = -400.0f;
        motors[i].pos_pid.limMaxInt = 400.0f;

        PIDController_Init(&motors[i].pos_pid);
    }

    xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 5120, NULL, 20, &pid_loop_task,0);

    ESP_LOGI(TAG, "PID application started successfully");
}  