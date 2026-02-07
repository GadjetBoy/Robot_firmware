#include "robot_controller.h"
ledc_timer_t LEDC_TIMER[] ={LEDC_TIMER_0,LEDC_TIMER_1};

uint8_t A=0;
uint8_t B=0;

 ledc_channel_t LEDC_A_CHANNEL[NUM_PINS_PER_MODULE]={
   LEDC_CHANNEL_0,
   LEDC_CHANNEL_1,
   LEDC_CHANNEL_2,
   LEDC_CHANNEL_3,
};
ledc_channel_t LEDC_B_CHANNEL[NUM_PINS_PER_MODULE]={
   LEDC_CHANNEL_0,
   LEDC_CHANNEL_1,
   LEDC_CHANNEL_2,
   LEDC_CHANNEL_3, 
 }; 
static int LEDC_OUTPUT_IO_A[NUM_PINS_PER_MODULE]={
    MOTOR1_PINA,
    MOTOR2_PINA, 
    MOTOR3_PINA,
    MOTOR4_PINA,  
};
static int LEDC_OUTPUT_IO_B[NUM_PINS_PER_MODULE]={
    MOTOR1_PINB,
    MOTOR2_PINB,
    MOTOR3_PINB,
    MOTOR4_PINB,
};

/********************************* setting up PWM modules and channels *************************/

void ledc_PWM_setup(void){

 for(int i=0; i<NUM_PINS_PER_MODULE/2;i++){
     // Prepare and Configure the LEDC PWM timer configurationA
     ledc_timer_config_t ledc_timerA = {
        .speed_mode       = LEDC_MODEA,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER[i],
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_USE_APB_CLK,
     };

     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerA));
    }

  // Prepare and Configure the LEDC PWM channel configurationA
  for( int i=0;i<NUM_PINS_PER_MODULE;i++){
     ledc_channel_config_t ledc_channelA = {
        .speed_mode     = LEDC_MODEA,
        .channel        = LEDC_A_CHANNEL[i],
        .timer_sel      = LEDC_TIMER[A], 
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_A[i],
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
         };
      ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelA));
      if(i==2){
         A=1;
        }
    }

 // Prepare and Configure the LEDC PWM timer configurationB
    for( int i=0; i<NUM_PINS_PER_MODULE/2;i++){
     // Prepare and Configure the LEDC PWM timer configurationA
     ledc_timer_config_t ledc_timerB = {
        .speed_mode       = LEDC_MODEB,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER[i],
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_USE_APB_CLK,
        };


     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerB));
    }

  // Prepare and Configure the LEDC PWM channel configurationBs
  for( int i=0;i<NUM_PINS_PER_MODULE;i++){
     ledc_channel_config_t ledc_channelB = {
        .speed_mode     = LEDC_MODEB,
        .channel        = LEDC_B_CHANNEL[i],
        .timer_sel      = LEDC_TIMER[B],
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_B[i],
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
         };
      ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelB));
      if(i==2){
         B=1;
        }
    }
     

}

void pwm_app_main(void)
{
 // Set the LEDC peripheral configuration
 ESP_LOGI(PWM_TAG, "setting up LEDC units and chaneels" );
 gpio_set_pull_mode(MOTOR1_PINA,GPIO_PULLUP_ONLY);
 gpio_set_pull_mode(MOTOR1_PINB,GPIO_PULLUP_ONLY);
 gpio_set_pull_mode(MOTOR2_PINA,GPIO_PULLUP_ONLY);
 gpio_set_pull_mode(MOTOR2_PINB,GPIO_PULLUP_ONLY);
 gpio_set_pull_mode(MOTOR3_PINA,GPIO_PULLUP_ONLY);
 gpio_set_pull_mode(MOTOR3_PINB,GPIO_PULLUP_ONLY);
 gpio_set_pull_mode(MOTOR4_PINA,GPIO_PULLUP_ONLY);
 gpio_set_pull_mode(MOTOR4_PINB,GPIO_PULLUP_ONLY);
 ledc_PWM_setup();
 ESP_LOGI(PWM_TAG, "LEDC unit setiing up completed\n");
}
