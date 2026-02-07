#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

/***********************************PWM control parameters*********************************/
#include "driver/ledc.h"
#include "hal/ledc_types.h" 

#define MOTOR1_PINA 21
#define MOTOR1_PINB 26

#define MOTOR2_PINA 22
#define MOTOR2_PINB 27

#define MOTOR3_PINA 23
#define MOTOR3_PINB 32

#define MOTOR4_PINA 25
#define MOTOR4_PINB 33

#define NUM_PINS_PER_CHANNEL    4//16
#define LEDC_A_TIMER              LEDC_TIMER_0
#define LEDC_B_TIMER              LEDC_TIMER_1
#define LEDC_MODEA              LEDC_LOW_SPEED_MODE
#define LEDC_MODEB              LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
//#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          1000 // Frequency in Hertz. Set frequency at 1 KHz
#define BIT_VALUE               4096 
extern ledc_channel_t LEDC_A_CHANNEL[NUM_PINS_PER_CHANNEL*2];
extern ledc_channel_t LEDC_B_CHANNEL[NUM_PINS_PER_CHANNEL*2];

void pwm_app_main(void);

/***********************************PID control parameters******************************/
#include "esp_timer.h"
#include <math.h>

#define NUM_MOTORS 4//8
#define PID_UPDATE_FREQ 500  //HZ
#define GEAR_RATIO 1000 //1:1000
#define MAX_MODES 5

#define MAX_CONTROL_SIGNAL (BIT_VALUE -1)  // Define MAX_CONTROL_SIGNAL based on system requirements
#define MAX_INTEGRAL 2000
#define MIN_CONTROL_SIGNAL (-(BIT_VALUE -1))

void pid_app_main(void);
void drive_motors(uint8_t pwm_channel,uint32_t pwm,int8_t direction);

/*********************************************PCNT param**********************************/
#include "driver/pulse_cnt.h"
#include "esp_attr.h"

#define ENCORDER1_PINA 4
#define ENCORDER1_PINB 16

#define ENCORDER2_PINA 13
#define ENCORDER2_PINB 17

#define ENCORDER3_PINA 14
#define ENCORDER3_PINB 18

#define ENCORDER4_PINA 15
#define ENCORDER4_PINB 19

#define PCNT_HIGH_LIMIT 20000
#define PCNT_LOW_LIMIT -20000
#define PCNT_VAL_UPDATE_FREQ 1000  //HZ

#define NUM_ENCORDERS NUM_MOTORS
#define EN_PPR 28.0f

extern volatile int32_t pcnt_count[NUM_ENCORDERS];
extern portMUX_TYPE pcnt_mux;

void pcnt_app_main(void);

/****************BLE param*********************************/

#include <inttypes.h>

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"

#define GATTS_TAG "BTserver"
#define TAG0      "BTwriteEvent"
#define PROFILE_A_APP_ID 0
#define PROFILE_NUM 1
#define TEST_DEVICE_NAME "ESP32_server"  
#define PREPARE_BUF_MAX_SIZE 32


#define GATT_PROFILE_SERVICE_INST 0

#define NUMBER_OF_MOTORS 8//NUM_MOTORS
#define SERVER_MAX_DATA_LEN 36
#define CLIENT_MAX_PACKET 20
#define NUMBER_OF_INTS 1
#define NUMBER_OF_FLOATS 8

extern portMUX_TYPE ble_mux;

volatile extern uint8_t int_val[NUMBER_OF_INTS];
volatile extern float float_val[NUMBER_OF_FLOATS];

extern void BLE_app_main(void);











































































































