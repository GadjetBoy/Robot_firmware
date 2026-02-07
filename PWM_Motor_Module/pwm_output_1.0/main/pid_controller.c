/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"*/

/*#include "esp_timer.h"
#include <math.h>*/

#include "pwm_pid.h"

//#define NUM_MOTORS 8
//#define PID_UPDATE_FREQ 100  //HZ

volatile float pos[NUM_MOTORS]={0};
 volatile float pos_set[NUM_MOTORS]={0};

//extern volatile float pos[NUM_MOTORS];
//extern volatile float pos_set[NUM_MOTORS];
volatile float duty[NUM_MOTORS];
TaskHandle_t PID_taskHandle = NULL;

typedef struct{
  //intialize values for PID
  float kp,ki,kd;
  float prev_pos;
  uint64_t prev_time;
  double integral;

  float prev_err;
  float setPoint;
  float current_speed;
  

  //intiialize 
}pid_control; 

pid_control motor_pid[NUM_MOTORS];

float compute_pid(pid_control *pid_param,float current_pos){

float error = (pid_param->setPoint)-current_pos;
int64_t current_t = esp_timer_get_time();
uint64_t time_differ = (current_t-(pid_param->prev_time))/1e6f;
//derivative
float derivative = (error-(pid_param->prev_err))/time_differ;
if(derivative<=0){    
   derivative = 1e-6f;
} 

//integral
 pid_param->integral += (error*time_differ);

//PID value
float U = pid_param->kp*error + pid_param->kd*derivative +(pid_param->ki)*(pid_param->integral);

pid_param->prev_err = error;
pid_param->prev_time = current_t;

 return U ;
}


void compute_pid_Task1(void *param){
const TickType_t xFrequency = pdMS_TO_TICKS(1000 / PID_UPDATE_FREQ);
TickType_t xLastWakeTime = xTaskGetTickCount();


while(1){
   for(int i=0;i<(NUM_MOTORS);i++){

	 motor_pid[i].setPoint = pos_set[i];

	 float control_signal = compute_pid(&motor_pid[i],pos[i]);

	 duty[i] = fabs(control_signal);
   if (duty[i] > 4095) {duty[i] = 4095;}

   if(i<NUM_PINS/2){

   // Set duty to 50%
   ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, LEDC_CHANNEL[i], duty[i]));
   // Update duty to apply the new value
   ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, LEDC_CHANNEL[i]));
  }
  else{
   ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, LEDC_CHANNEL[i], duty[i]));
   // Update duty to apply the new value
   ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, LEDC_CHANNEL[i]));

  }



}
vTaskDelayUntil(&xLastWakeTime, xFrequency);
}

}

void app_main(void)
{
  esp_err_t esp_timer_early_init(void);
  pwm_app_main();

  for(int i = 0;i < NUM_MOTORS;i++){
  	motor_pid[i].kp=1.0;
 		motor_pid[i].ki=0.0;
 		motor_pid[i].kd=0.0;
 		motor_pid[i].integral =0.0;
 		motor_pid[i].prev_time = esp_timer_get_time();
 		motor_pid[i].setPoint=0.0;
 		motor_pid[i].prev_err=pos_set[i];
  }

xTaskCreatePinnedToCore( compute_pid_Task1,
                          "pid_task",
                          1028,
                          NULL,
                          5,
                          &PID_taskHandle,
                          tskNO_AFFINITY
                          );

}
