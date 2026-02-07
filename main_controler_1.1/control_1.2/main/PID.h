#pragma once

#include "control_1.2.h"
#include "PWM.h"
// ====================== PID Structure ======================
typedef struct {
    float Kp, Ki, Kd;
    float T;                  
    float tau;                
    float limMin, limMax;     
    float limMinInt, limMaxInt; 

    float integrator;
    float prevError;
    float differentiator;
    float prev_differentiator;  // Previous derivative filter state
    float prevMeasurement;
    float out;

    bool firstUpdate;
} PIDController;

// ====================== Motor Structure ======================
typedef struct {
    uint8_t m_num;
    ledc_channel_t pwm_channelA;
    ledc_channel_t pwm_channelB;
    uint8_t pinA;
    uint8_t pinB;
    int direction, prev_dir;

    int32_t current_position;   
    int32_t target_position;
    int32_t last_enc_val;

    int32_t pwm;
    bool err_change;
    bool active;
    bool at_target;
    float abs_error;
    int32_t last_pwm;
    bool flip_dir;
 
    PIDController pos_pid;    
    PIDController speed_pid;  
} Motor;

typedef struct {
    uint8_t motor_index;
    int32_t target;
    bool activate;
} motor_command_t;

typedef struct {
    float error;
    float proportional;
    float derivative;
    float integrator;
} PID_debugger_log;

extern Motor motor_log[NUM_MOTORS];

#define PWM_MAX         1023    // full-scale PWM
#define PWM_MIN         350//1010  // minimum duty to move motor
#define PWM_STOP        0       // below this -> fully stop (coast)
#define TOL             70    // position tolerance (encoder ticks)
#define RAMP_ZONE       1000    // Distance from target to begin sine ramp down
#define RAMP_STEPS      50      // Number of steps for ramp-up & ramp-down
#define COUNTS_PER_REV  28      // adjust for your encoder
#define POS_DT          0.08f   // 100 Hz position loop


void pid_app_main(void);
void motor_set(uint8_t i,int32_t target,bool set);
