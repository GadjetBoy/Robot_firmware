#pragma once 
//====================== Common header files ==============================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_slave.h"
#include "esp_timer.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"

//======================= PID unit ========================================
#include <math.h>
#define  NUM_MOTORS  8
extern volatile int encorder_val[NUM_MOTORS];
void pid_app_main(void);
extern portMUX_TYPE encoder_mux;
// Function to safely read encoder values
int safe_read_encoder(int index);

//========================PWM unit ========================================

#include "driver/ledc.h"
#include "hal/ledc_types.h" 

#define LEDC_A_TIMER            LEDC_TIMER_0
#define LEDC_B_TIMER            LEDC_TIMER_1
#define LEDC_MODEA              LEDC_LOW_SPEED_MODE
#define LEDC_MODEB              LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define NUM_PINS_PER_MODULE     NUM_MOTORS

#define LEDC_FREQUENCY          20000 // Frequency in Hertz. Set frequency at 1 KHz

extern ledc_channel_t LEDC_A_CHANNEL[NUM_PINS_PER_MODULE];
extern ledc_channel_t LEDC_B_CHANNEL[NUM_PINS_PER_MODULE];
void pwm_app_main(void);

#define GPIO_MODE GPIO_PULLDOWN_ONLY

typedef struct{
	bool active ;
}motor_state;

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
} PIDController;

extern int pos_target[NUM_MOTORS];

// ====================== State Machine ======================
typedef enum {
    MOTOR_STOPPED,
    MOTOR_MOVING,
    MOTOR_RAMP_UP,
    MOTOR_RAMP_DOWN_SINE
} MotorState;


// ====================== Motor Structure ======================
typedef struct {
    int m_num;
    ledc_channel_t pwm_channelA;
    ledc_channel_t pwm_channelB;
    int direction, prev_dir;

    float current_position;   
    float target_position;
    int last_enc_val;
    int freeze_counter;

    float current_speed;      
    float target_speed;

    int pwm;
    bool err_change;
    bool active;
    bool at_target;
    int max_error_seen;
    int last_pwm;
    bool flip_dir;
    uint8_t ramp_step;
    MotorState state;
 

    PIDController pos_pid;    
    PIDController speed_pid;  
} Motor;

extern Motor motors[NUM_MOTORS];
void motor_set(Motor *m,float target,bool set);



#define PHASE_A_DONE_BIT   (1 << 0)
#define PHASE_B_DONE_BIT   (1 << 1)

extern EventGroupHandle_t crawl_event_group;

extern uint8_t phaseA_remaining ;
extern uint8_t phaseB_remaining ;

void mot_command_app_main(void);

//=================================BLE===================================
#include <inttypes.h>
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define BLE_TAG "BLE UNIT"

#define GATTS_CHAR_UUID_TX 0xFF01  
#define GATTS_CHAR_UUID_RX 0xFF02
#define GATTS_NUM_HANDLE_A 7

#define TX_DATA_SIZE (NUM_MOTORS*sizeof(float))

#define PROFILE_A_APP_ID 0
#define PROFILE_NUM 1
#define TEST_DEVICE_NAME "ESP32"  

#define GATT_PROFILE_SERVICE_INST 0

#define SERVER_MAX_DATA_LEN 68
#define CLIENT_MAX_PACKET 20
#define NUMBER_OF_BYTES 1
#define NUMBER_OF_FLOATS 16


extern portMUX_TYPE ble_mux;

volatile extern uint8_t byte_val[NUMBER_OF_BYTES];
volatile extern float float_val[NUMBER_OF_FLOATS];

extern void BLE_app_main(void);
