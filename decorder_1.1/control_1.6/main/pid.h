#pragma once

#include "control_1.6.h"

// ====================== PID Structure ======================
typedef struct {
    float Kp, Ki, Kd;
    float T;                                
    float limMin, limMax;     
    float limMinInt, limMaxInt; 

    float proportional,differentiator,integrator;
    float error,prevError;
    float prevMeasurement;
    float output,lastOutput;

} PIDController;

// ====================== Motor Structure ======================
typedef struct {
    uint8_t m_num;
    float target_position;
    int current_position;   
    int last_enc_val;
    bool active;
 
    PIDController pos_pid;    
} Motor;

typedef struct {
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
#define COUNTS_PER_REV  28      // adjust for your encoder
#define MIN_DUTY        (PWM_MIN * 0.6f)  // 60% of minimum motion duty
#define STOP_THRESHOLD   30
#define PID_UPDATE_RATE_HZ 5000 //HZ
#define PID_DT (1.0f / PID_UPDATE_RATE_HZ)  //seconds
#define PID_UPDATE_RATE_US PID_DT*1000000.0f // microseconds

// ====================== SAFE DOUBLE BUFFER ======================
extern volatile float motor_cmd_buffer[2][NUM_MOTORS];
extern volatile uint8_t current_write_index;  // PID reads from this
extern portMUX_TYPE pid_buffer_swap_mux;

void pid_app_main(void);
extern TaskHandle_t pid_loop_task;

