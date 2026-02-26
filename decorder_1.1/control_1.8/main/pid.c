#include "cpg.h"
#include "ble.h"

#define TAG_PID "PID_CTRL"

#define PID_MAX_I 500.0f

volatile DRAM_ATTR Motor Update_PID ={0}; 

// ====================== PID Functions ======================
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->output = 0.0f;
}

// Dynamic gain update based on frequency (called periodically)
void update_PID_gain(void) {

    //if (xSemaphoreTake(pid_params_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
    taskENTER_CRITICAL(&ble_mux);
       Update_PID.pos_pid.Kp = pid_ble_params.Kp;
       Update_PID.pos_pid.Ki = pid_ble_params.Ki;
       Update_PID.pos_pid.Kd = pid_ble_params.Kd;
    taskEXIT_CRITICAL(&ble_mux);
        //xSemaphoreGive(pid_params_mutex);
    //}
    
    float freq_scale = fminf(CPG_frequency / 0.1f, 2.5f);  // FIXED: Cap at 2.5x (less aggressive than 3.0)
    for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].pos_pid.Kp = Update_PID.pos_pid.Kp * fminf(1.0f + 0.3f * freq_scale, 2.5f);  // Cap eff Kp<=5
            motors[i].pos_pid.Kd = Update_PID.pos_pid.Kd * (1.0f + 0.6f * freq_scale);  // Milder Kd scale
            motors[i].pos_pid.Ki = fmaxf(Update_PID.pos_pid.Ki * 0.8f, 0.005f);  // FIXED: Floor at 0.005, less reduction
            motors[i].pos_pid.limMaxInt = 300.0f / freq_scale;  // FIXED: Tighter (200→150) to limit windup             
    }
}


void PIDController_Update(PIDController *pid, float setpoint, int measurement, uint8_t i) {
   
    pid->prevError = pid->error;
    pid->lastOutput = pid->output;
    pid->prevMeasurement = (float)measurement;
 
    pid->error = setpoint - (float)measurement;
    //ESP_LOGE(TAG, "CPG[%1d] (out: %.2f)",i ,pid->error);

    float abs_error = fabsf(pid->error);
    float abs_output = fabsf(pid->lastOutput);
    float kp_scale = 1.0f;

    if (abs_error < TOL  || abs_output > MIN_DUTY) {  // Near zero or saturated (peak)
      kp_scale = 0.5f + 0.5f * fminf(abs_error / (TOL), abs_output / PWM_MAX); // FIXED: Blend error/output for peak focus
    }
    pid->proportional = pid->Kp * kp_scale * pid->error;

    // Derivative (add output blend if needed)
    float kd_scale = 0.10f;
    if (abs_output > MIN_DUTY) kd_scale = 1.5f;  // FIXED: Higher (1.5→1.8) for low Kd
    pid->differentiator = pid->Kd * kd_scale * (pid->error - pid->prevError) / pid->T;

    //integral term
    pid->integrator += pid->Ki *pid->error*pid->T;
    // Clamp integrator
    if (pid->integrator > pid->limMaxInt){
        pid->integrator = pid->limMaxInt;
    }
    if (pid->integrator < pid->limMinInt){
        pid->integrator = pid->limMinInt;
    }
    if(fabs(pid->error)<=STOP_THRESHOLD*5){
      pid->integrator *= 0.9f; // decay
    }
    //calculate PID output
    pid->output = pid->proportional +pid->differentiator+pid->integrator;
    
    float raw_output = pid->proportional + pid->differentiator + pid->integrator;

    // Anti-windup: Back-calculate integrator if saturated
    if (raw_output > pid->limMax) {
     pid->integrator -= (raw_output - pid->limMax) * pid->Ki * pid->T; // Undo excess
     pid->output = pid->limMax;
    } else if (raw_output < pid->limMin) {
     pid->integrator -= (raw_output - pid->limMin) * pid->Ki * pid->T;
     pid->output = pid->limMin;
    } else {
     pid->output = raw_output;
    }

    /* Lighter smoothing in IDLE when error is large - faster homing */
    if (cpg_run_mode == CPG_MODE_HOMING && fabsf(pid->error) > TOL) {
        pid->output = 0.9f * pid->output + 0.1f * pid->lastOutput;
    } else {
        pid->output = 0.2f * pid->output + 0.8f * pid->lastOutput;
    }
}

// ====================== Motor Setup ======================
void pid_app_main(void) {
    ESP_LOGI(TAG_PID, "Starting PID application...");
   
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i + 1;
        motors[i].target_position = 0.0f;
        motors[i].current_position = 0;
        motors[i].active = false;
        motors[i].pos_pid.Kp = 2.00f;
        motors[i].pos_pid.Ki = 0.00f;
        motors[i].pos_pid.Kd = 0.00f;
        motors[i].pos_pid.T = PID_DT;
        motors[i].pos_pid.limMin = -PWM_MAX;
        motors[i].pos_pid.limMax = PWM_MAX;
        motors[i].pos_pid.limMinInt = -PID_MAX_I;
        motors[i].pos_pid.limMaxInt = PID_MAX_I;
        PIDController_Init(&motors[i].pos_pid);
    }
    //xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 5120, NULL, 20, &pid_loop_task,1);
    ESP_LOGI(TAG_PID, "PID application started successfully");
}