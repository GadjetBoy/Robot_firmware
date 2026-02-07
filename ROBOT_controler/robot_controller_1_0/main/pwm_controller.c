#include "robot_controller.h"

 ledc_channel_t LEDC_CHANNEL[NUM_PINS]={
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
    MOTOR1_PINA,
    MOTOR2_PINA, 
    MOTOR3_PINA,
    MOTOR4_PINA, 
    MOTOR5_PINA,
    MOTOR6_PINA, 
    MOTOR7_PINA,
    MOTOR8_PINA, 
    MOTOR1_PINB,
    MOTOR2_PINB,
    MOTOR3_PINB,
    MOTOR4_PINB,
    MOTOR5_PINB,
    MOTOR6_PINB,
    MOTOR7_PINB,
    MOTOR8_PINB,

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
