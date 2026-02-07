#include "control_1.3.h"
#include "PID.h"
#include "motor_command.h"
#include "UART.h"

#define TAG "PID"
#define PID_QUEUE_LENGTH 10

// ====================== Globals ======================
static Motor motors[NUM_MOTORS];
QueueHandle_t motor_command_queue;
static QueueHandle_t pid_log_data_que;
static PID_debugger_log pid_log[NUM_MOTORS];
static portMUX_TYPE pwm_mux = portMUX_INITIALIZER_UNLOCKED;

// Forward declarations
void drive_motor(Motor* m);
void stop_motor(Motor *m);

// ====================== PID Functions ======================
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->firstUpdate = true;
    pid->out = 0.0f;
    pid->prev_differentiator = 0.0f;  // Initialize missing member
}


float PIDController_Update(PIDController *pid, int32_t setpoint, int32_t measurement, uint8_t i) {
    float error = (float)setpoint - (float)measurement;

    // Proportional term
    float proportional = pid->Kp * error;

    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    // Clamp integrator
    if (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
    else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

    float derivative;

    if (!pid->firstUpdate) {

      float delta_measurement = (float)measurement - pid->prevMeasurement;

      derivative = -(2.0f * pid->Kd * delta_measurement
                   - (2.0f * pid->tau - pid->T) * pid->prev_differentiator)  // Note the change: + to -
                     /(2.0f * pid->tau + pid->T);

        
     } else {

        derivative = 0;
        pid->firstUpdate = false;
    }

    pid->differentiator = derivative;
    pid->prev_differentiator = derivative;

    pid->out = proportional + pid->integrator + derivative;

    if (pid->out > pid->limMax) pid->out = pid->limMax;
    else if (pid->out < pid->limMin) pid->out = pid->limMin;

    pid->prevError = error;
    pid->prevMeasurement = (float)measurement;

    pid_log[i] = (PID_debugger_log){
        .error = error,
        .proportional = proportional,
    };

    return pid->out;
}

// ====================== Motor Control ======================
void drive_motor(Motor* m) {
    taskENTER_CRITICAL(&pwm_mux);
    if (m->direction == -1) {
        ledc_set_duty(LEDC_MODEA, m->pwm_channelA, 0);
        ledc_set_duty(LEDC_MODEB, m->pwm_channelB, m->pwm);
    
    } else {
        ledc_set_duty(LEDC_MODEA, m->pwm_channelA, m->pwm);
        ledc_set_duty(LEDC_MODEB, m->pwm_channelB, 0);
    }
    ledc_update_duty(LEDC_MODEA, m->pwm_channelA);
    ledc_update_duty(LEDC_MODEB, m->pwm_channelB);
    taskEXIT_CRITICAL(&pwm_mux);
}

void stop_motor(Motor *m) {
    taskENTER_CRITICAL(&pwm_mux);
    ledc_set_duty(LEDC_MODEA, m->pwm_channelA, PWM_STOP);
    ledc_set_duty(LEDC_MODEB, m->pwm_channelB, PWM_STOP);
    
    ledc_update_duty(LEDC_MODEA, m->pwm_channelA);
    ledc_update_duty(LEDC_MODEB, m->pwm_channelB);
    taskEXIT_CRITICAL(&pwm_mux);
}
static inline void apply_break(Motor *m){
 taskENTER_CRITICAL(&pwm_mux);
 ESP_ERROR_CHECK(ledc_stop(LEDC_MODEA, m->pwm_channelA, 1)); // Set idle level to HIGH
 ESP_ERROR_CHECK(ledc_stop(LEDC_MODEB, m->pwm_channelB, 1)); // Set idle level to HIGH
 taskEXIT_CRITICAL(&pwm_mux);
}

// ====================== Motor Functions ======================
void motor_update_position(Motor *m, int counts) {
    m->current_position = (float)counts;
}

void motor_set(uint8_t i, int32_t target, bool set) {
    motor_command_t cmd = {i, target, set};
    xQueueSend(motor_command_queue, &cmd, portMAX_DELAY);
}

// ====================== Position Loop ======================
void position_loop_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS((int)(POS_DT * 1000));

    uint8_t stall_count[NUM_MOTORS] = {0};
    static motor_command_t cmd;

    while (1) {
        while (xQueueReceive(motor_command_queue, &cmd, 0) == pdTRUE) {
            motors[cmd.motor_index].target_position = cmd.target;
            motors[cmd.motor_index].active = cmd.activate;
            if (cmd.activate) {
                //PIDController_Init(&motors[cmd.motor_index].pos_pid);
            }
        }

        //update_PID_gain();

        for (int i = 0; i < NUM_MOTORS; i++) {
            taskENTER_CRITICAL(&encoder_mux);
            int curr_counts = encorder_val[i];
            taskEXIT_CRITICAL(&encoder_mux);

            motors[i].current_position = curr_counts;

            if (!motors[i].active) {
                stop_motor(&motors[i]);
                apply_break(&motors[i]); // Apply brake when idle
                continue; 
            }

           /* ESP_LOGW("PID", "M[%d]: curr_counts=%" PRId32 ", current_position=%" PRId32,
                     motors[i].m_num, (int32_t)curr_counts, (int32_t)motors[i].current_position);*/

            float error = (float)motors[i].target_position - (float)motors[i].current_position;
            //ESP_LOGW("PID", "M[%d]: error=%.1f", motors[i].m_num, error);

            float abs_error = fabsf(error);

            float pid_output = PIDController_Update(&motors[i].pos_pid,
                                                    motors[i].target_position,
                                                    motors[i].current_position, i);

            int new_dir = (error > 0) ? 1 : -1;
            if (motors[i].direction != new_dir) motors[i].direction = new_dir;

            int pwm_magnitude = (int)fabsf(pid_output);

            if (pwm_magnitude < PWM_MIN && abs_error > TOL) {
                pwm_magnitude = PWM_MIN;
            } else if (pwm_magnitude > PWM_MAX) {
                pwm_magnitude = PWM_MAX;
            }

            if (abs_error < TOL * 2 && pwm_magnitude < PWM_MIN / 2) {
                pwm_magnitude = PWM_MIN * 0.45f;
            }

            if (abs_error > TOL) {
                if (motors[i].current_position == motors[i].last_enc_val) {
                    stall_count[i]++;
                    if (stall_count[i] >= 5) {
                        pwm_magnitude = PWM_MAX * 0.7f;
                        ESP_LOGI("TARGET", "Motor %d kick start", i);
                    }
                } else {
                    stall_count[i] = 0;
                }
            }

            motors[i].pwm = pwm_magnitude;
            motors[i].last_pwm = pwm_magnitude;
            motors[i].last_enc_val = motors[i].current_position;

            drive_motor(&motors[i]);
        }

             Motor motors_copy[NUM_MOTORS];
             memcpy(motors_copy, motors, sizeof(motors));
             //xQueueOverwrite(pid_log_data_que, motors_copy);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ====================== PID Logging ======================

// ====================== Motor Setup ======================
void pid_app_main(void) {
    ESP_LOGI(TAG, "Starting PID application...");

    // Configure logging - reduce noise but don't disable
    //esp_log_level_set("*", ESP_LOG_WARN);
    //esp_log_level_set("PID", ESP_LOG_INFO);
    
    motor_command_queue = xQueueCreate(PID_QUEUE_LENGTH, sizeof(motor_command_t));
    if (!motor_command_queue) {
        ESP_LOGE(TAG, "Creating motor command queue failed");
        return;
    }

    pid_log_data_que = xQueueCreate(1, sizeof(motors));
    if (!pid_log_data_que) {
        ESP_LOGE(TAG, "Creating PID log queue failed");
        return;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i + 1;
        motors[i].target_position = 0;
        motors[i].current_position = 0;
        motors[i].prev_dir = 0;
        motors[i].pwm_channelA = LEDC_A_CHANNEL[i];
        motors[i].pwm_channelB = LEDC_B_CHANNEL[i];
        motors[i].pinA = GPIOArray[i].PINA;
        motors[i].pinB = GPIOArray[i].PINB;
        motors[i].active = false;
        motors[i].at_target = false;

        motors[i].pos_pid.Kp = 0.50f;
        motors[i].pos_pid.Ki = 0.00f;
        motors[i].pos_pid.Kd = 0.001f;

        motors[i].pos_pid.T = POS_DT;
        motors[i].pos_pid.tau = 0.02f;
        motors[i].pos_pid.limMin = -PWM_MAX;
        motors[i].pos_pid.limMax = PWM_MAX;
        motors[i].pos_pid.limMinInt = -1000;
        motors[i].pos_pid.limMaxInt = 1000;

        PIDController_Init(&motors[i].pos_pid);
    }

    xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 5120, NULL, 9, NULL, 0);
   // xTaskCreatePinnedToCore(PID_logging_task, "PID_logging_task", 4096, NULL, 4, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "PID application started successfully");
}