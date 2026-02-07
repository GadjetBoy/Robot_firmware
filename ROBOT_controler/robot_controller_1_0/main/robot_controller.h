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

#define MOTOR1_PINA 17
#define MOTOR1_PINB 25

#define MOTOR2_PINA 18
#define MOTOR2_PINB 26

#define MOTOR3_PINA 19
#define MOTOR3_PINB 27

#define MOTOR4_PINA 20
#define MOTOR4_PINB 28

#define MOTOR5_PINA 21
#define MOTOR5_PINB 29

#define MOTOR6_PINA 22
#define MOTOR6_PINB 30

#define MOTOR7_PINA 23
#define MOTOR7_PINB 31

#define MOTOR8_PINA 24
#define MOTOR8_PINB 32

#define NUM_PINS                16
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODEA              LEDC_LOW_SPEED_MODE
#define LEDC_MODEB              LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
//#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 1 KHz
#define BIT_VALUE               1024 // 
extern ledc_channel_t LEDC_CHANNEL[NUM_PINS];

void pwm_app_main(void);

/***********************************PID control parameters******************************/
#include "esp_timer.h"
#include <math.h>

#define NUM_MOTORS 8
#define PID_UPDATE_FREQ 500  //HZ
#define GEAR_RATIO 1000

#define MAX_CONTROL_SIGNAL 2000.0f // Define MAX_CONTROL_SIGNAL based on system requirements
#define MAX_INTEGRAL 100

/*********************************************PCNT param**********************************/
#include "driver/pulse_cnt.h"
#include "esp_attr.h"

#define ENCORDER1_PINA 1
#define ENCORDER1_PINB 9

#define ENCORDER2_PINA 2
#define ENCORDER2_PINB 10

#define ENCORDER3_PINA 3
#define ENCORDER3_PINB 11

#define ENCORDER4_PINA 4
#define ENCORDER4_PINB 12

#define ENCORDER5_PINA 5
#define ENCORDER5_PINB 13

#define ENCORDER6_PINA 6
#define ENCORDER6_PINB 14

#define ENCORDER7_PINA 7
#define ENCORDER7_PINB 15

#define ENCORDER8_PINA 8
#define ENCORDER8_PINB 16


#define PCNT_HIGH_LIMIT 20000
#define PCNT_LOW_LIMIT -20000
#define PCNT_VAL_UPDATE_FREQ 1000  //HZ

#define NUM_ENCORDERS NUM_MOTORS
extern volatile int Data_Buff[NUM_ENCORDERS];
extern portMUX_TYPE mux;

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
#define PROFILE_A_APP_ID 0
#define PROFILE_NUM 1
#define TEST_DEVICE_NAME "ESP32_server"  

#define GATT_PROFILE_SERVICE_INST 0

extern volatile uint8_t rx_buffer[NUM_MOTORS];
extern portMUX_TYPE mux1;

extern void BLE_app_main(void);