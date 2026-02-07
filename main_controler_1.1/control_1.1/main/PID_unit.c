#include "control_1.1.h"
#define PWM_MAX         1023    // full-scale PWM
#define PWM_MIN         400//1010  // minimum duty to move motor
#define PWM_STOP        0       // below this -> fully stop (coast)
#define TOL             50     // position tolerance (encoder ticks)
#define RAMP_ZONE       1000    // Distance from target to begin sine ramp down
#define RAMP_STEPS      50      // Number of steps for ramp-up & ramp-down
#define COUNTS_PER_REV  28      // adjust for your encoder
#define POS_DT          0.05f   // 100 Hz position loop

#define SPEED_DT    0.05f     // 500 Hz speed loop


// ====================== Globals ======================
Motor motors[NUM_MOTORS];
int prev_counts[NUM_MOTORS] = {0};

// Forward declarations
void drive_motor(Motor* m);
void stop_motor(Motor *m);

// ====================== PID Functions ======================
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.00f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.00f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

static portMUX_TYPE pwm_mux = portMUX_INITIALIZER_UNLOCKED;


float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    
    // Proportional term
    float proportional = pid->Kp * error;

    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
    
    // Clamp integrator
    if (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
    else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

    // === DERIVATIVE (with proper initialization and bounds) ===
    float derivative;

    // Only calculate derivative if we have valid previous data
    if (pid->prevMeasurement != 0 || pid->prevError != 0){
     float delta_measurement = measurement - pid->prevMeasurement;
    
     // Low-pass filtered derivative
     derivative = -(2.0f * pid->Kd * delta_measurement
                 + (2.0f * pid->tau - pid->T) * pid->prev_differentiator)
              / (2.0f * pid->tau + pid->T);
    
     // Limit the derivative to prevent extreme spikes
     float max_derivative = pid->limMax * 0.5f; // Limit to 50% of max output
     if (derivative > max_derivative) derivative = max_derivative;
     else if (derivative < -max_derivative) derivative = -max_derivative;

    } else {
     // First run - initialize derivative to zero
     derivative = 0;
     //pid->initialized = true;
    }

    pid->differentiator = derivative;
    pid->prev_differentiator = pid->differentiator;

    // Calculate final output
    pid->out = proportional + pid->integrator + pid->differentiator;
    
    // Clamp the PID output
    if (pid->out > pid->limMax) pid->out = pid->limMax;
    else if (pid->out < pid->limMin) pid->out = pid->limMin;

    pid->prevError = error;
    pid->prevMeasurement = measurement;

    ESP_LOGE("PID", "Error=%.1f, P=%.1f, I=%.1f,D=%.1f,Out=%.0f", 
             error, proportional, pid->integrator,pid->differentiator,pid->out);

    return pid->out;
}

// ====================== Motor Control Functions ======================
void drive_motor(Motor* m){
    if (m->direction == -1) {
        ledc_set_duty(LEDC_MODEA, m->pwm_channelA, 0);
        ledc_update_duty(LEDC_MODEA, m->pwm_channelA);

        ledc_set_duty(LEDC_MODEB, m->pwm_channelB, m->pwm);
        ledc_update_duty(LEDC_MODEB, m->pwm_channelB);

    }else if(m->direction == 0){
        ledc_set_duty(LEDC_MODEA, m->pwm_channelA, PWM_MAX );
        ledc_update_duty(LEDC_MODEA, m->pwm_channelA);

        ledc_set_duty(LEDC_MODEB, m->pwm_channelB, PWM_MAX );
        ledc_update_duty(LEDC_MODEB, m->pwm_channelB);
    
    } else {
        ledc_set_duty(LEDC_MODEA, m->pwm_channelA, m->pwm);
        ledc_update_duty(LEDC_MODEA, m->pwm_channelA);

        ledc_set_duty(LEDC_MODEB, m->pwm_channelB, 0);
        ledc_update_duty(LEDC_MODEB, m->pwm_channelB);
    }
}

void stop_motor(Motor *m){
    ledc_set_duty(LEDC_MODEA, m->pwm_channelA, PWM_STOP);
    ledc_update_duty(LEDC_MODEA, m->pwm_channelA);

    ledc_set_duty(LEDC_MODEB, m->pwm_channelB, PWM_STOP);
    ledc_update_duty(LEDC_MODEB, m->pwm_channelB);
}


// ====================== Motor Functions ======================
void motor_update_speed(Motor *m, int delta_counts, float dt) {
    m->current_speed = ((float)delta_counts / COUNTS_PER_REV) / dt;
}

void motor_update_position(Motor *m, int counts) {
    m->current_position = (float)counts;
}

void motor_set(Motor *m,float target,bool set){

   m->target_position = (float)target;
   m->active = set;
   if (set) {
        PIDController_Init(&m->pos_pid); // Reset PID when starting new movement
    }
}

int safe_read_encoder(int index) {
    if (index < 0 || index >= NUM_MOTORS) return 0;
    
    int value;
    taskENTER_CRITICAL(&encoder_mux);
    value = encorder_val[index];
    taskEXIT_CRITICAL(&encoder_mux);
    return value;
}

void ramp_down_sine(Motor *m, float initial_pwm, float distance_to_target) {
    // Calculate a factor from 1.0 down to 0 using a quarter-sine wave
    float ramp_factor = sinf((distance_to_target / RAMP_ZONE) * M_PI_2);
    m->pwm = (int)(initial_pwm * ramp_factor);
    
    if (m->pwm <= PWM_MIN || distance_to_target < TOL) {
        m->pwm = 0;
        m->state = MOTOR_STOPPED;
    } else {
        m->state = MOTOR_RAMP_DOWN_SINE;
    }
}

void ramp_up_sine(Motor *m, float target_pwm) {
    // Calculate a value between 0 and 1 using a quarter-sine wave
    float ramp_factor = sinf((float)m->ramp_step / RAMP_STEPS * M_PI_2);
    m->pwm = (int)(PWM_MIN + (target_pwm - PWM_MIN) * ramp_factor);
    
    // Clamp the value to ensure it doesn't go below PWM_MIN
    if (m->pwm < PWM_MIN) m->pwm = PWM_MIN;

    if (m->ramp_step >= RAMP_STEPS) {
        m->pwm = target_pwm;
        m->state = MOTOR_MOVING;
    } else {
        m->state = MOTOR_RAMP_UP;
    }
    m->ramp_step++;
}

// ====================== Improved Position Loop ======================
void position_loop_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS((int)(POS_DT * 1000));

    uint8_t stall_count[NUM_MOTORS]={0};

    while (1) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            int curr_counts = safe_read_encoder(i);
            motors[i].current_position = (float)curr_counts;

            if (!motors[i].active) {
                motors[i].at_target = false;
                continue;
            }

            // Check target tolerance first
            float error = motors[i].target_position - motors[i].current_position;
            float abs_error = fabsf(error);
            
            if (abs_error < TOL) {
                taskENTER_CRITICAL(&pwm_mux);
                stop_motor(&motors[i]);
                taskEXIT_CRITICAL(&pwm_mux);
                motors[i].at_target = true;
                motors[i].active = false;
                PIDController_Init(&motors[i].pos_pid); // Reset PID when target reached
                xEventGroupSetBits(crawl_event_group, PHASE_A_DONE_BIT);
                ESP_LOGI("TARGET", "Motor %d reached target: %.1f", i, motors[i].current_position);
                continue;
            }

            // --- PID calculation ---
            float pid_output = PIDController_Update(&motors[i].pos_pid,
                                                  motors[i].target_position,
                                                  motors[i].current_position);

            // Determine direction based on error sign
            int new_dir = (error > 0) ? 1 : -1;
            
            // Only update direction if it actually changes
            if (motors[i].direction != new_dir) {
                motors[i].direction = new_dir;
            }

            // Calculate PWM magnitude from PID output
            int pwm_magnitude = (int)fabsf(pid_output);
            
            // Apply PWM deadband and clamping
            if (pwm_magnitude < PWM_MIN && abs_error > TOL) {
                pwm_magnitude = PWM_MIN; // Minimum power to overcome friction
            } else if (pwm_magnitude > PWM_MAX) {
                pwm_magnitude = PWM_MAX;
            }
            
            // If very close to target, use minimum PWM
            if (abs_error < TOL * 2) {
             if (pwm_magnitude < PWM_MIN / 2){
                 pwm_magnitude = PWM_MIN / 2;
                }
            }

            if(abs_error > TOL && motors[i].current_position==motors[i].last_enc_val){
              stall_count[i]++;
              if(stall_count[i] == 5){
                pwm_magnitude = PWM_MAX*0.6;
                ESP_LOGI("TARGET", "Motor %d kik start",i);
                stall_count[i] = 0;
              }
            }
            
            motors[i].pwm = pwm_magnitude;
            motors[i].last_pwm = pwm_magnitude;
            motors[i].last_enc_val = motors[i].current_position;

            // --- Drive motor safely ---
            taskENTER_CRITICAL(&pwm_mux);
            drive_motor(&motors[i]);
            taskEXIT_CRITICAL(&pwm_mux);

            // Debug logging for oscillation detection
            if (fabs(pid_output) >= PWM_MAX * 0.9) {
                ESP_LOGW("OSCILLATION", "Motor %d PID saturated: %.0f, Error: %.1f", 
                         i, pid_output, error);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void PID_logging_task(void* arg){
 while(1){

         for(int i = 0;i<NUM_MOTORS;i++){
             if (motors[i].active){
                 ESP_LOGI("CASCADE", "M[%d]: PosT=%.1f Pos=%.1f| PWM=%d Dir=%d",
                 motors[i].m_num,motors[i].target_position,motors[i].current_position,
                 motors[i].pwm,motors[i].direction);
                }
            }

      vTaskDelay(pdMS_TO_TICKS(100)); 
    } 
}

void oscillation_debug_task(void* arg){
    static int last_positions[NUM_MOTORS] = {0};
    static int oscillation_count[NUM_MOTORS] = {0};
    static TickType_t last_check = 0;
    
    while(1) {
        TickType_t now = xTaskGetTickCount();
        if (now - last_check > pdMS_TO_TICKS(1000)) { // Check every second
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (motors[i].active) {
                    int current_pos = (int)motors[i].current_position;
                    if (abs(current_pos - last_positions[i]) < 50) { // Small movement
                        oscillation_count[i]++;
                        if (oscillation_count[i] > 5) { // Stuck for 5 seconds
                            ESP_LOGW("STUCK", "Motor %d may be stuck at position %d", i, current_pos);
                            oscillation_count[i] = 0;
                        }
                    } else {
                        oscillation_count[i] = 0;
                    }
                    last_positions[i] = current_pos;
                }
            }
            last_check = now;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}



// ====================== Improved Motor Setup ======================
void pid_app_main(void) {
    TaskHandle_t position_task = NULL;

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i;
        motors[i].target_position = 0;
        motors[i].current_position = 0;
        motors[i].prev_dir = 0;
        motors[i].pwm_channelA = LEDC_A_CHANNEL[i];
        motors[i].pwm_channelB = LEDC_B_CHANNEL[i];
        motors[i].active = false;

        // Much more conservative PID gains to prevent oscillation
        motors[i].pos_pid.Kp = 0.1f;    // Reduced from 1.0
        motors[i].pos_pid.Ki = 0.01f;   // Reduced from 0.10  
        motors[i].pos_pid.Kd = 0.005f;   // Reduced from 0.50
        
        motors[i].pos_pid.T = POS_DT;
        motors[i].pos_pid.tau = 0.02f;
        motors[i].pos_pid.limMin = -PWM_MAX;
        motors[i].pos_pid.limMax = PWM_MAX;
        motors[i].pos_pid.limMinInt = -100;   // Much smaller integrator limits
        motors[i].pos_pid.limMaxInt = 100;
        
        PIDController_Init(&motors[i].pos_pid);
    }

    xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 4096, NULL, 22, &position_task, 1);
    xTaskCreatePinnedToCore(PID_logging_task, "PID_logging_task", 3072, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(oscillation_debug_task, "osc_debug", 2048, NULL, 3, NULL, tskNO_AFFINITY);
}