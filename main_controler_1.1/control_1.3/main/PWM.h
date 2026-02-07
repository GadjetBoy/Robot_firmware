#pragma once

#include "control_1.3.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h" 

#define LEDC_A_TIMER            LEDC_TIMER_0
#define LEDC_B_TIMER            LEDC_TIMER_1
#define LEDC_MODEA              LEDC_LOW_SPEED_MODE
#define LEDC_MODEB              LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define NUM_PINS_PER_MODULE     NUM_MOTORS

#define LEDC_FREQUENCY          20000 // Frequency in Hertz. Set frequency at 1 KHz
#define GPIO_MODE GPIO_PULLDOWN_ONLY

typedef struct{
   uint8_t PINA;
   uint8_t PINB;
}Motor_pins;

extern ledc_channel_t LEDC_A_CHANNEL[NUM_PINS_PER_MODULE];
extern ledc_channel_t LEDC_B_CHANNEL[NUM_PINS_PER_MODULE];

extern Motor_pins GPIOArray[];

void pwm_app_main(void);



