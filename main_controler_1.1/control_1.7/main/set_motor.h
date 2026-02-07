#pragma once

#include "control_1.7.h"
#include "pwm.h"

// ====================== Motor Command Structure ======================

typedef struct {
    uint8_t m_num;
    ledc_channel_t pwm_channelA;
    ledc_channel_t pwm_channelB;
    int direction;
    
    uint32_t pwm;
    uint32_t last_pwm;

} motor_Command_t;

#define PWM_MAX         1023.0f    // full-scale PWM
#define PWM_MIN         500.0f//1010  // minimum duty to move motor
#define PWM_STOP        0       // below this -> fully stop (coast)
#define TOL             1000    // position tolerance (encoder ticks)
#define MIN_DUTY        (PWM_MIN * 0.6f)  // 60% of minimum motion duty
#define STOP_THRESHOLD   30

#define PWM_QUEUE_LENGTH 1
#define MOTOR_UPDATE_RATE_Hz 5000  // 2kHz—match master CPG/PID rate
#define MOTOR_UPDATE_RATE_US (1000000.0f/MOTOR_UPDATE_RATE_Hz) // 2kHz—match master CPG/PID rate

extern TaskHandle_t pid_loop_task;
extern esp_timer_handle_t motor_timer_handle;

extern motor_Command_t motors[NUM_MOTORS];
extern motor_Command_t motor_log[NUM_MOTORS];

void set_motor_app_main(void);
