#include "control_1.6.h"
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

motor_Command_t motors[NUM_MOTORS];

extern motor_Command_t motor_log[NUM_MOTORS];

#define PWM_MAX         1023.0f    // full-scale PWM
#define PWM_MIN         400.0f//1010  // minimum duty to move motor
#define PWM_STOP        0       // below this -> fully stop (coast)
#define TOL             1000    // position tolerance (encoder ticks)
#define MIN_DUTY        (PWM_MIN * 0.6f)  // 60% of minimum motion duty
#define STOP_THRESHOLD   30

extern TaskHandle_t pid_loop_task;

#define TAG "MOTOR_SET"
#define PWM_QUEUE_LENGTH 1

void set_motor_app_main(void);
