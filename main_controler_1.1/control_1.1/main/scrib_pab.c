#include "control_1.1.h"

#define PWM_MAX     255
#define PWM_MIN     10
#define COUNTS_PER_REV 28   // adjust for your encoder
#define SPEED_DT    0.05f     // 500 Hz speed loop
#define POS_DT      0.01f     // 100 Hz position loop
#define TOL         5

static portMUX_TYPE pwm_mux = portMUX_INITIALIZER_UNLOCKED;


//====================== Globals ======================
Motor motors[NUM_MOTORS];
int prev_counts[NUM_MOTORS] = {0};

void drive_motor(Motor* m);
void stop_motor(Motor *m);


void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.05f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.50f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    float proportional = pid->Kp * error;

    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
    if (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
    else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                          + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);

    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) pid->out = pid->limMax;
    else if (pid->out < pid->limMin) pid->out = pid->limMin;

    pid->prevError = error;
    pid->prevMeasurement = measurement;

    return pid->out;
}



// ====================== Motor Functions ======================
void motor_update_speed(Motor *m, int delta_counts, float dt) {
    m->current_speed = ((float)delta_counts / COUNTS_PER_REV) / dt;
}

void motor_update_position(Motor *m, int counts) {
    m->current_position = (float)counts;
}
void motor_set(Motor *m,float target,bool set){

   m->target_position = target;
   m->active = set;
}

void motor_cascade_update(Motor *m) {
    float output = PIDController_Update(&m->speed_pid, m->target_speed, m->current_speed);

    if (output < 0) {
        m->direction = -1;
    } else {
        m->direction = 1;
    }

    float pwm = fabs(output);
    if (pwm >= PWM_MAX) m->pwm = PWM_MAX;
    else if (pwm < PWM_MIN) m->pwm = PWM_MIN;
    else m->pwm = pwm;

    if (m->direction != m->prev_dir) {
        stop_motor(m);
        m->prev_dir = m->direction;
        m->speed_pid.integrator = 0.0f;
    }

    ESP_LOGI("CASCADE", "M[%d]: PosT=%.1f Pos=%.1f | SpdT=%.2f Spd=%.2f | PWM=%d Dir=%d",
             m->m_num, m->target_position, m->current_position,
             m->target_speed, m->current_speed, m->pwm, m->direction);

    taskENTER_CRITICAL(&pwm_mux);

    if(fabs( m->target_position - m->current_position)<TOL){
      if(m->active){
         stop_motor(m);
         m->at_target = true; 

         if ((m->m_num) % 2 == 0) {  // even motor
              if (--phaseA_remaining == 0) {
                 xEventGroupSetBits(crawl_event_group, PHASE_A_DONE_BIT);
                }
            } else { // odd motor
                  if (--phaseB_remaining == 0) {
                     xEventGroupSetBits(crawl_event_group, PHASE_B_DONE_BIT);
                    } 
                } 

         m->speed_pid.integrator = 0.0f;
         m->speed_pid.prevError = 0.0f;
         m->pos_pid.prevError = 0.0f;

         m->active = false ;

        }
    }
    else {
     drive_motor(m);
     m->active = true ;
    }

    taskEXIT_CRITICAL(&pwm_mux);
    
}

void drive_motor(Motor *m){

  if(m->direction == -1 ){
      ledc_set_duty(LEDC_MODEA , m->pwm_channelA,0);
      ledc_update_duty(LEDC_MODEA,m->pwm_channelA);

      ledc_set_duty(LEDC_MODEB,m->pwm_channelB,m->pwm);
      ledc_update_duty(LEDC_MODEB,m->pwm_channelB);
    }
  else{
      ledc_set_duty(LEDC_MODEA ,m->pwm_channelA,m->pwm);
      ledc_update_duty(LEDC_MODEA,m->pwm_channelA);

      ledc_set_duty(LEDC_MODEB,m->pwm_channelB,0);
      ledc_update_duty(LEDC_MODEB,m->pwm_channelB);
   }
   
   return ;
}
void stop_motor(Motor *m){
   
    ledc_set_duty(LEDC_MODEA ,m->pwm_channelA,0);
    ledc_update_duty(LEDC_MODEA,m->pwm_channelA);

    ledc_set_duty(LEDC_MODEB,m->pwm_channelB,0);
    ledc_update_duty(LEDC_MODEB,m->pwm_channelB);

    return;

}
//========================set_motor======================


// ====================== FreeRTOS Tasks ======================
void position_loop_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount ();
    const TickType_t xFrequency = pdMS_TO_TICKS((int)(POS_DT * 1000));

    while (1) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            float spd_target = PIDController_Update(&motors[i].pos_pid,
                                                    motors[i].target_position,
                                                    motors[i].current_position);
            motors[i].target_speed = spd_target;
        }
     xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }

}

void speed_loop_task(void *arg) {
    
    TickType_t xLastWakeTime = xTaskGetTickCount ();
    const TickType_t xFrequency = pdMS_TO_TICKS((int)(SPEED_DT * 1000));

    while (1) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            int curr_counts = encorder_val[i]; /* TODO: read_encoder_counts(i) */
            int delta = curr_counts - prev_counts[i];
            prev_counts[i] = curr_counts;

            motor_update_position(&motors[i], curr_counts);
            motor_update_speed(&motors[i], delta, SPEED_DT);
            if(motors[i].active){
                motor_cascade_update(&motors[i]);
            }
            
        }
        xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

// ====================== Main ======================
void pid_app_main(void) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i;
        motors[i].target_position = 0;
        motors[i].current_position = 0;
        motors[i].prev_dir = 0;
        motors[i].pwm_channelA = LEDC_A_CHANNEL[i];
        motors[i].pwm_channelB = LEDC_B_CHANNEL[i];

        // Outer position PID
        motors[i].pos_pid.Kp = 2.0f; motors[i].pos_pid.Ki = 0.1f; motors[i].pos_pid.Kd = 0.0f;
        motors[i].pos_pid.T = POS_DT;
        motors[i].pos_pid.tau = 0.02f;
        motors[i].pos_pid.limMin = -200; motors[i].pos_pid.limMax = 200;
        motors[i].pos_pid.limMinInt = -10; motors[i].pos_pid.limMaxInt = 10;
        PIDController_Init(&motors[i].pos_pid);

        // Inner speed PID
        motors[i].speed_pid.Kp = 10.0f; motors[i].speed_pid.Ki = 1.0f; motors[i].speed_pid.Kd = 0.1f;
        motors[i].speed_pid.T = SPEED_DT;
        motors[i].speed_pid.tau = 0.01f;
        motors[i].speed_pid.limMin = -PWM_MAX; motors[i].speed_pid.limMax = PWM_MAX;
        motors[i].speed_pid.limMinInt = -100; motors[i].speed_pid.limMaxInt = 100;
        PIDController_Init(&motors[i].speed_pid);
    }

    xTaskCreatePinnedToCore(position_loop_task, 
                            "pos_loop",
                             4096, 
                             NULL, 
                             20, 
                             NULL, 
                             0);
    xTaskCreatePinnedToCore(speed_loop_task, 
                            "spd_loop", 
                             4096, 
                             NULL, 
                             21, 
                             NULL, 
                             0);


}



//================================================================================================


#include "control_1.1.h"

#define PWM_MAX    1023    // full-scale PWM
#define PWM_MIN     440    // minimum duty to move motor
#define PWM_STOP    420  // below this → fully stop
#define TOL          10     // position tolerance (encoder ticks)
#define TOL_SCALE    50
#define MIN_ERROR   100     // minimum assumed max error (safety)
#define DECAY_RATE   0.99f // decay factor per cycle (slowly lowers max_error_seen)
#define DECAY_FACTOR 4     // 25%
#define RAMP_STEP   15    
#define COUNTS_PER_REV 28   // adjust for your encoder
#define SPEED_DT    0.05f     // 500 Hz speed loop
#define POS_DT      0.05f     // 100 Hz position loop

static portMUX_TYPE pwm_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
  uint8_t up;
  uint8_t down;
} steps;

static steps step[NUM_MOTORS]={0};


//====================== Globals ======================
Motor motors[NUM_MOTORS];
int prev_counts[NUM_MOTORS] = {0};

void drive_motor(Motor* m);
void stop_motor(Motor *m);


void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.00f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.00f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    
     float error = setpoint - measurement;
     float proportional = pid->Kp * error;

     pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
     if (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
     else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

     pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                          + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);

     pid->out = proportional + pid->integrator + pid->differentiator;
 
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    return pid->out;
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
}

int safe_read_encoder(int index) {
    if (index < 0 || index >= NUM_MOTORS) return 0;
    
    int value;
    taskENTER_CRITICAL(&encoder_mux);
    value = encorder_val[index];
    taskEXIT_CRITICAL(&encoder_mux);
    return value;
}

void drive_motor(Motor *m){

  if(m->direction == -1 ){
      ledc_set_duty(LEDC_MODEA , m->pwm_channelA,0);
      ledc_update_duty(LEDC_MODEA,m->pwm_channelA);

      ledc_set_duty(LEDC_MODEB,m->pwm_channelB,m->pwm);
      ledc_update_duty(LEDC_MODEB,m->pwm_channelB);
    }
  else{
      ledc_set_duty(LEDC_MODEA ,m->pwm_channelA,m->pwm);
      ledc_update_duty(LEDC_MODEA,m->pwm_channelA);

      ledc_set_duty(LEDC_MODEB,m->pwm_channelB,0);
      ledc_update_duty(LEDC_MODEB,m->pwm_channelB);
   }
   
   return ;
}
void stop_motor(Motor *m){
   
    ledc_set_duty(LEDC_MODEA ,m->pwm_channelA,PWM_MAX);
    ledc_update_duty(LEDC_MODEA,m->pwm_channelA);

    ledc_set_duty(LEDC_MODEB,m->pwm_channelB,PWM_MAX);
    ledc_update_duty(LEDC_MODEB,m->pwm_channelB);

    return;

}
//========================set_motor======================

void position_loop_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS((int)(POS_DT * 1000));

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].max_error_seen = MIN_ERROR;
        motors[i].last_pwm = 0;
    }

    while (1) {

        for (int i = 0; i < NUM_MOTORS; i++) {
            // --- Error calculation ---
            float error = motors[i].target_position - motors[i].current_position;
            float abs_error = fabsf(error);

            float tol_final = (TOL*TOL_SCALE);

            // --- Auto-learn max error with decay ---
            if (abs_error > motors[i].max_error_seen) {
                motors[i].max_error_seen = abs_error;
            } else {
                // Apply decay so value slowly decreases over time
                motors[i].max_error_seen = (int)(motors[i].max_error_seen * DECAY_RATE);
                if (motors[i].max_error_seen < MIN_ERROR)
                    motors[i].max_error_seen = MIN_ERROR;
            }

            int curr_counts = safe_read_encoder(i);
            prev_counts[i] = curr_counts;
            motor_update_position(&motors[i], curr_counts);

            if (!motors[i].active) continue;  // skip inactive motors
            
            // --- Reached target? ---
            if (abs_error < tol_final){

                // Ramp down gradually
                if (step[i].down < RAMP_STEP ) {
                    // Calculate target PWM for smooth ramp-down
                 float ramp_ratio = (float)(RAMP_STEP - step[i].down) / RAMP_STEP;
                 int target_pwm = (int)(motors[i].last_pwm * ramp_ratio);
        
                  // Ensure we don't go below stop threshold during ramp
                  if (target_pwm < PWM_STOP) {
                     target_pwm = PWM_STOP;
                    }
                   
                  motors[i].pwm = target_pwm;
                  step[i].down ++;
    
                 // Only drive if we have meaningful PWM
                 if (motors[i].pwm >= PWM_MIN) {
                      taskENTER_CRITICAL(&pwm_mux);
                      drive_motor(&motors[i]);
                      taskEXIT_CRITICAL(&pwm_mux);
                    } else {
                      taskENTER_CRITICAL(&pwm_mux);
                      stop_motor(&motors[i]);
                      taskEXIT_CRITICAL(&pwm_mux);
                    }
                    ESP_LOGI("RAMP SEQUENCE", "M[%d]:REMAIN ERROR =%.1f",motors[i].m_num,abs_error);
                } else {

                    // Fully stop
                    step[i].down = 0;
                    taskENTER_CRITICAL(&pwm_mux);
                    stop_motor(&motors[i]);
                    taskEXIT_CRITICAL(&pwm_mux);

                    motors[i].at_target = true;
                    motors[i].active = false;

                    // Signal crawl phase done
                    //if ((motors[i].m_num) % 2 == 0) {  
                        //if (--phaseA_remaining == 0) {
                            xEventGroupSetBits(crawl_event_group, PHASE_A_DONE_BIT);
                       // }
                   // } else { 
                        //if (--phaseB_remaining == 0) {
                          //  xEventGroupSetBits(crawl_event_group, PHASE_B_DONE_BIT);
                        //}
                   // }

                    // Reset PID states
                    motors[i].pos_pid.integrator = 0.0f;
                    motors[i].pos_pid.prevError = 0.0f;

                    ESP_LOGE("RAMP SEQUENCE", "M[%d]: motor STOPED",motors[i].m_num);
   
                }
            } 

            else if (motors[i].flip_dir == true && step[i].up < RAMP_STEP) {
              float ramp_ratio = (float)(step[i].up + 1) / RAMP_STEP;
              int target_pwm = (int)(PWM_STOP + (motors[i].last_pwm - PWM_STOP) * ramp_ratio);

              if (target_pwm < PWM_STOP) target_pwm = PWM_STOP;

              motors[i].pwm = target_pwm;
              step[i].up++;

             if (motors[i].pwm >= PWM_MIN) {
                  taskENTER_CRITICAL(&pwm_mux);
                  drive_motor(&motors[i]);
                  taskEXIT_CRITICAL(&pwm_mux);
             } else {
                 taskENTER_CRITICAL(&pwm_mux);
                 stop_motor(&motors[i]);
                 taskEXIT_CRITICAL(&pwm_mux);
                }

             motors[i].last_pwm = motors[i].pwm;


              ESP_LOGI("RAMP UP SEQUENCE", "M[%d]:REMAIN ERROR =%.1f", motors[i].m_num, abs_error);

              if (step[i].up >= RAMP_STEP) {
                 motors[i].flip_dir = false;
                 step[i].up = 0;
                }
            }

            // --- Not yet at target ---
            else {
                step[i].down = 0; // reset ramp sequence if moving again

            float pwm_out = PIDController_Update(&motors[i].pos_pid,
                                                 motors[i].target_position,
                                                 motors[i].current_position);

            // Direction + deadband
             if (fabsf(pwm_out) < 50.0f) {  
                 pwm_out = 0;  // deadband to avoid jitter
                 motors[i].pwm = pwm_out;
                 motors[i].prev_dir = 0; // Reset direction memory
                 motors[i].last_pwm =0;
                }

             else{ // Scale PWM safely
                  float max_pid_output = motors[i].pos_pid.Kp * motors[i].max_error_seen;
                  if (max_pid_output < 1.0f) max_pid_output = 1.0f;

                  float pwm_scaled = (pwm_out / max_pid_output) * PWM_MAX;
                  motors[i].pwm = (int)fabsf(pwm_scaled);
            
                  // Apply PWM thresholds
                 if (motors[i].pwm > PWM_MAX) motors[i].pwm = PWM_MAX;
                 if (motors[i].pwm < PWM_MIN && motors[i].pwm > 0) motors[i].pwm = PWM_MIN;

                 motors[i].direction = (pwm_out >= 0) ? 1 : -1;
                 motors[i].last_pwm = motors[i].pwm;

                 // If direction changed, stop briefly first
                 if (motors[i].prev_dir != motors[i].direction) {
                     motors[i].flip_dir = true;
                     taskENTER_CRITICAL(&pwm_mux);
                     stop_motor(&motors[i]);
                     taskEXIT_CRITICAL(&pwm_mux);
                     motors[i].prev_dir = motors[i].direction;
                     motors[i].pwm = 0;
                     motors[i].last_pwm =0;
                     motors[i].pos_pid.integrator = 0.0f;
                    }
                }
 
                taskENTER_CRITICAL(&pwm_mux);
                drive_motor(&motors[i]);
                taskEXIT_CRITICAL(&pwm_mux);
            }
        }

        // Run loop at fixed period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

    

void PID_logging_task(void* arg){
 while(1){

         for(int i = 0;i<NUM_MOTORS;i++){
             ESP_LOGI("CASCADE", "M[%d]: PosT=%.1f Pos=%.1f| PWM=%d Dir=%d",
             motors[i].m_num,motors[i].target_position,motors[i].current_position,
             motors[i].pwm,motors[i].direction);
            }

      vTaskDelay(pdMS_TO_TICKS(1000)); 
    } 
}

// ====================== Main ======================
void pid_app_main(void) {

 TaskHandle_t position_task = NULL;

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i;
        motors[i].target_position = 0;
        motors[i].current_position = 0;
        motors[i].prev_dir = 0;
        motors[i].pwm_channelA = LEDC_A_CHANNEL[i];
        motors[i].pwm_channelB = LEDC_B_CHANNEL[i];

        // Outer position PID
        motors[i].pos_pid.Kp = 1.0f; motors[i].pos_pid.Ki = 0.90f; motors[i].pos_pid.Kd = 0.4f;
        motors[i].pos_pid.T = POS_DT;
        motors[i].pos_pid.tau = 0.02f;
        motors[i].pos_pid.limMin =  -1023; motors[i].pos_pid.limMax =  1023;
        motors[i].pos_pid.limMinInt = - 600; motors[i].pos_pid.limMaxInt = 600;
        //motors[i].target_position = 60000;
        PIDController_Init(&motors[i].pos_pid);

        // Inner speed PID
        motors[i].speed_pid.Kp = 5.0f; motors[i].speed_pid.Ki = 1.0f; motors[i].speed_pid.Kd = 0.1f;
        motors[i].speed_pid.T = SPEED_DT;
        motors[i].speed_pid.tau = 0.01f;
        motors[i].speed_pid.limMin = -PWM_MAX; motors[i].speed_pid.limMax = PWM_MAX;
        motors[i].speed_pid.limMinInt = -100; motors[i].speed_pid.limMaxInt = 100;
        PIDController_Init(&motors[i].speed_pid);
    }

    xTaskCreatePinnedToCore(position_loop_task, 
                            "pos_loop",
                             4096, 
                             NULL, 
                             23, 
                             &position_task, 
                             0);

    xTaskCreatePinnedToCore(PID_logging_task, 
                            "PID_logging_task",
                             3072, 
                             NULL, 
                             5, 
                             NULL, 
                             tskNO_AFFINITY);

}




esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp) {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);

                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK) {
                    ESP_LOGE(BLE_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            } 
            else {
                ESP_LOGE(BLE_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK){
                return;
            }