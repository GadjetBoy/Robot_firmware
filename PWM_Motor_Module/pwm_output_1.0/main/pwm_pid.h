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

#include "esp_timer.h"
#include <math.h>

#include "driver/ledc.h"
#include "hal/ledc_types.h"

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

//PWM control parameters 

#define NUM_PINS                16
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODEA              LEDC_LOW_SPEED_MODE
#define LEDC_MODEB              LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT // Set duty resolution to 13 bits
//#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 KHz

static ledc_channel_t LEDC_CHANNEL[NUM_PINS];

//PID controlparameters

#define NUM_MOTORS 8
#define PID_UPDATE_FREQ 100  //HZ

void pwm_app_main(void);