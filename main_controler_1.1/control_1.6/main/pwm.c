#include "pwm.h"

#define TAG "LEDC_PWM_UNIT"

#define MOTOR1_PINA 32
#define MOTOR1_PINB 33

#define MOTOR2_PINA 25
#define MOTOR2_PINB 26

#define MOTOR3_PINA 27
#define MOTOR3_PINB 14

#define MOTOR4_PINA 23
#define MOTOR4_PINB 22

#define MOTOR5_PINA 21
#define MOTOR5_PINB 19

#define MOTOR6_PINA 18
#define MOTOR6_PINB 5

#define MOTOR7_PINA 17
#define MOTOR7_PINB 16

#define MOTOR8_PINA 4
#define MOTOR8_PINB 2

Motor_pins GPIOArray[] = {
  {MOTOR1_PINA,MOTOR1_PINB},
  {MOTOR2_PINA,MOTOR2_PINB},
  {MOTOR3_PINA,MOTOR3_PINB},
  {MOTOR4_PINA,MOTOR4_PINB},
  {MOTOR5_PINA,MOTOR5_PINB},
  {MOTOR6_PINA,MOTOR6_PINB},
  {MOTOR7_PINA,MOTOR7_PINB},
  {MOTOR8_PINA,MOTOR8_PINB}
};

 ledc_channel_t LEDC_A_CHANNEL[NUM_PINS_PER_MODULE] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
    LEDC_CHANNEL_7,
};

ledc_channel_t LEDC_B_CHANNEL[NUM_PINS_PER_MODULE] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
    LEDC_CHANNEL_7,
};
static int LEDC_OUTPUT_IO_A[NUM_PINS_PER_MODULE]={
    MOTOR1_PINA,
    MOTOR2_PINA, 
    MOTOR3_PINA,
    MOTOR4_PINA,
    MOTOR5_PINA,
    MOTOR6_PINA, 
    MOTOR7_PINA,
    MOTOR8_PINA,  
};
static int LEDC_OUTPUT_IO_B[NUM_PINS_PER_MODULE]={
    MOTOR1_PINB,
    MOTOR2_PINB,
    MOTOR3_PINB,
    MOTOR4_PINB,
    MOTOR5_PINB,
    MOTOR6_PINB,
    MOTOR7_PINB,
    MOTOR8_PINB,
};

/********************************* setting up PWM modules and channels *************************/

void ledc_PWM_setup(void){

 
     // Prepare and Configure the LEDC PWM timer configurationA
     ledc_timer_config_t ledc_timerA = {
        .speed_mode       = LEDC_MODEA,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_A_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_USE_APB_CLK,
     };

     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerA));


  // Prepare and Configure the LEDC PWM channel configurationA
  for( int i=0;i<NUM_PINS_PER_MODULE;i++){
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

 // Prepare and Configure the LEDC PWM timer configurationB
    
     // Prepare and Configure the LEDC PWM timer configurationA
     ledc_timer_config_t ledc_timerB = {
        .speed_mode       = LEDC_MODEB,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_B_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_USE_APB_CLK,
        };


     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timerB));
 

  // Prepare and Configure the LEDC PWM channel configurationBs
  for( int i=0;i<NUM_PINS_PER_MODULE;i++){
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
 for (int i = 0;i<NUM_MOTORS;i++){
    gpio_reset_pin(GPIOArray[i].PINA); 
    gpio_reset_pin(GPIOArray[i].PINB);

    gpio_set_direction(GPIOArray[i].PINA, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIOArray[i].PINB, GPIO_MODE_OUTPUT);

    gpio_set_pull_mode(GPIOArray[i].PINA,GPIO_MODE);
    gpio_set_pull_mode(GPIOArray[i].PINB,GPIO_MODE);
 }
 vTaskDelay(pdMS_TO_TICKS(10));
 ledc_PWM_setup();
 vTaskDelay(pdMS_TO_TICKS(100));
 ESP_LOGI(TAG, "LEDC unit setiing up completed\n");
}
