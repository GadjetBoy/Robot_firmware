#pragma once

#include "control_1.5.h"
#include "PWM.h"

// ====================== PID Structure ======================
typedef struct {
    float Kp, Ki, Kd;
    float T;                                
    float limMin, limMax;     
    float limMinInt, limMaxInt; 

    float proportional,differentiator,integrator;
    float error,prevError;
    float prevMeasurement;
    float out,lastOutput;

} PIDController;

// ====================== Motor Structure ======================
typedef struct {
    uint8_t m_num;
    ledc_channel_t pwm_channelA;
    ledc_channel_t pwm_channelB;
    int direction;
    
    float target_position;
    int32_t current_position;   
    int32_t last_enc_val;
    uint32_t pwm;
    uint32_t last_pwm;
    bool active;
 
    PIDController pos_pid;    
} Motor;

typedef struct {
    uint8_t motor_index;
    float motor_target;  // Array to hold the target positions for all motors
    bool activate;
} motor_Command_t;

typedef struct {
    float error;
    float proportional;
    float derivative;
    float integrator;
    int32_t encorder_val;
} PID_debugger_log;

extern Motor motor_log[NUM_MOTORS];
extern QueueHandle_t motor_command_queue;

#define PWM_MAX         1023.0f    // full-scale PWM
#define PWM_MIN         400.0f//1010  // minimum duty to move motor
#define PWM_STOP        0       // below this -> fully stop (coast)
#define TOL             1000    // position tolerance (encoder ticks)
#define RAMP_ZONE       1000    // Distance from target to begin sine ramp down
#define COUNTS_PER_REV  28      // adjust for your encoder
#define PID_UPDATE_RATE_MS  1      // 200 Hz position loop
#define POS_DT          ((float)PID_UPDATE_RATE_MS / 1000.0f)
#define MIN_DUTY        (PWM_MIN * 0.6f)  // 60% of minimum motion duty
#define STOP_THRESHOLD   30

void pid_app_main(void);
extern TaskHandle_t pid_loop_task;

