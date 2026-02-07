#include "robot_controller.h"

#define TAG "LEDC_UNIT"

 ledc_channel_t LEDC_A_CHANNEL[NUM_PINS_PER_CHANNEL*2]={
   LEDC_CHANNEL_0,
   LEDC_CHANNEL_1,
   LEDC_CHANNEL_2,
   LEDC_CHANNEL_3,
   LEDC_CHANNEL_4,
   LEDC_CHANNEL_5,
   LEDC_CHANNEL_6,
   LEDC_CHANNEL_7
};
ledc_channel_t LEDC_B_CHANNEL[NUM_PINS_PER_CHANNEL*2]={
   LEDC_CHANNEL_0,
   LEDC_CHANNEL_1,
   LEDC_CHANNEL_2,
   LEDC_CHANNEL_3,
   LEDC_CHANNEL_4,
   LEDC_CHANNEL_5,
   LEDC_CHANNEL_6,
   LEDC_CHANNEL_7
};
static int LEDC_OUTPUT_IO_A[NUM_PINS_PER_CHANNEL]={
    MOTOR1_PINA,
    MOTOR2_PINA, 
    MOTOR3_PINA,
    MOTOR4_PINA, 
   /* MOTOR5_PINA,
    MOTOR6_PINA, 
    MOTOR7_PINA,
    MOTOR8_PINA,*/ 
    
};
static int LEDC_OUTPUT_IO_B[NUM_PINS_PER_CHANNEL]={
    MOTOR1_PINB,
    MOTOR2_PINB,
    MOTOR3_PINB,
    MOTOR4_PINB,
    /*MOTOR5_PINB,
    MOTOR6_PINB,
    MOTOR7_PINB,
    MOTOR8_PINB,*/
};







void ledc_PWM_setup(void){

 // Prepare and then apply the LEDC PWM timer configurationA
    ledc_timer_config_t ledc_timerA = {
        .speed_mode       = LEDC_MODEA,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_A_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_USE_APB_CLK,
    };

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerA));

  // Prepare and then apply the LEDC PWM channel configuration
  for(int i=0;i<NUM_PINS_PER_CHANNEL;i++){
    ledc_channel_config_t ledc_channelA = {
        .speed_mode     = LEDC_MODEA,
        .channel        = LEDC_A_CHANNEL[i],
        .timer_sel      = LEDC_A_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_A[i],
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
 ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelA));
}

// Prepare and then apply the LEDC PWM timer configurationB
    ledc_timer_config_t ledc_timerB = {
        .speed_mode       = LEDC_MODEB,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_B_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_USE_APB_CLK,
    };

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerB));

  // Prepare and then apply the LEDC PWM channel configuration
  for(int i=0;i<NUM_PINS_PER_CHANNEL;i++){
    ledc_channel_config_t ledc_channelB = {
        .speed_mode     = LEDC_MODEB,
        .channel        = LEDC_B_CHANNEL[i],
        .timer_sel      = LEDC_B_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_B[i],
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
 ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelB));
}

}



void pwm_app_main(void)
{
 // Set the LEDC peripheral configuration
 ESP_LOGI(TAG, "setting up LEDC units and chaneels" );
 gpio_set_pull_mode(MOTOR1_PINA,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(MOTOR1_PINB,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(MOTOR2_PINA,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(MOTOR2_PINB,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(MOTOR3_PINA,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(MOTOR3_PINB,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(MOTOR4_PINA,GPIO_PULLDOWN_ONLY);
 gpio_set_pull_mode(MOTOR4_PINB,GPIO_PULLDOWN_ONLY);
 ledc_PWM_setup();
 ESP_LOGI(TAG, "LEDC unit setiing up completed\n");
}
