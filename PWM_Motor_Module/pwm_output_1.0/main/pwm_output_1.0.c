/*#include <stdio.h>
#include "driver/pulse_cnt.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"*/

/*#include "driver/ledc.h"
#include "hal/ledc_types.h"*/

#include "pwm_pid.h"


/*#define NUM_MOTORS              16
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODEA              LEDC_LOW_SPEED_MODE
#define LEDC_MODEA              LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT // Set duty resolution to 13 bits
//#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 KHz

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
#define ENCORDER8_PINB 16*/


static ledc_channel_t LEDC_CHANNEL[NUM_PINS]={
   LEDC_CHANNEL_0,
   LEDC_CHANNEL_1,
   LEDC_CHANNEL_2,
   LEDC_CHANNEL_3,
   LEDC_CHANNEL_4,
   LEDC_CHANNEL_5,
   LEDC_CHANNEL_6,
   LEDC_CHANNEL_7
};
static int LEDC_OUTPUT_IO[NUM_PINS]={
    ENCORDER1_PINA,
    ENCORDER2_PINA, 
    ENCORDER3_PINA,
    ENCORDER4_PINA, 
    ENCORDER5_PINA,
    ENCORDER6_PINA, 
    ENCORDER7_PINA,
    ENCORDER8_PINA, 
    ENCORDER1_PINB,
    ENCORDER2_PINB,
    ENCORDER3_PINB,
    ENCORDER4_PINB,
    ENCORDER5_PINB,
    ENCORDER6_PINB,
    ENCORDER7_PINB,
    ENCORDER8_PINB,

};
void ledc_PWM_setup(void){

 // Prepare and then apply the LEDC PWM timer configurationA
    ledc_timer_config_t ledc_timerA = {
        .speed_mode       = LEDC_MODEA,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_USE_APB_CLK,
    };

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerA));

  // Prepare and then apply the LEDC PWM channel configuration
  for(int i=0;i<NUM_PINS/2;i++){
    ledc_channel_config_t ledc_channelA = {
        .speed_mode     = LEDC_MODEA,
        .channel        = LEDC_CHANNEL[i],
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO[i],
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
 ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelA));
}

// Prepare and then apply the LEDC PWM timer configurationB
    ledc_timer_config_t ledc_timerB = {
        .speed_mode       = LEDC_MODEB,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_USE_APB_CLK,
    };

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerB));

  // Prepare and then apply the LEDC PWM channel configuration
  for(int i=NUM_PINS/2;i<NUM_PINS;i++){
    ledc_channel_config_t ledc_channelB = {
        .speed_mode     = LEDC_MODEB,
        .channel        = LEDC_CHANNEL[i],
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO[i],
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
 ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelB));
}

}



void pwm_app_main(void)
{
 // Set the LEDC peripheral configuration
 ledc_PWM_setup();
 
}
