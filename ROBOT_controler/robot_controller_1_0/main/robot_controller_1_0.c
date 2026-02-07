#include "robot_controller.h"

portMUX_TYPE pwm_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
  uint32_t duty;   // Absolute duty value (0 to BIT_VALUE)
  bool direction;  // true = forward (Channel A), false = reverse (Channel B)
} motor_cmd_t;

static volatile motor_cmd_t motor_commands[NUM_MOTORS];  // Shared buffer

#define PWM_TAG "PWM"
//array to hold the set values for motors sent over BLE
volatile float pos_set[NUM_MOTORS]={0};

//declare task handle
TaskHandle_t PID_taskHandle = NULL;
TaskHandle_t pwm_taskHandle = NULL;

typedef struct{
  //intialize values for PID
  float kp,ki,kd;
  float prev_pos;
  uint64_t prev_time;
  float integral;

  float prev_err;
  float setPoint;
  float current_speed;
  float max_control_signal_val;

}pid_control; 

pid_control motor_pid[NUM_MOTORS];

float compute_pid(pid_control *pid_param,float current_pos){
 //claculaet error
 float error = (pid_param->setPoint)-current_pos;
 //calculate the elapsed time
 int64_t current_t = esp_timer_get_time();
 //calculate dt value
 uint64_t dt = (current_t-(pid_param->prev_time))/1e6f;
 if(dt<=0){    
   dt = 1e-6f;
  } 
 //derivative
 float derivative = (error-(pid_param->prev_err))/(float)dt;
 
 //integral
 pid_param->integral += (float)(error * dt); // Use float
 
 // Anti-windup clamping
 if (pid_param->integral > MAX_INTEGRAL) {
    pid_param->integral = MAX_INTEGRAL;
  }
 else if (pid_param->integral < -MAX_INTEGRAL) {
    pid_param->integral = -MAX_INTEGRAL;
  }

 float U = pid_param->kp * error + pid_param->kd * derivative + pid_param->ki * pid_param->integral;

 // Clamp control signal
 U = (U > MAX_CONTROL_SIGNAL) ? MAX_CONTROL_SIGNAL : 
      (U < -MAX_CONTROL_SIGNAL) ? -MAX_CONTROL_SIGNAL : U;
 //store values 
 pid_param->prev_err = error;
 pid_param->prev_time = current_t;
 ESP_LOGI(PWM_TAG, "control_signal %f",U);
 return U ;
}



void compute_pid_Task(void *pvParam){
 const TickType_t xFrequency = pdMS_TO_TICKS(1000 / PID_UPDATE_FREQ);
 TickType_t xLastWakeTime = xTaskGetTickCount();

 float current_pos[NUM_MOTORS];
 float map_val = BIT_VALUE / MAX_CONTROL_SIGNAL; // Precompute outside loop
 float pos_set_local[NUM_MOTORS];
 
 while(1){

  // Read all setpoints in one critical section
    taskENTER_CRITICAL(&mux1);
    memcpy(pos_set_local,(void*) pos_set, sizeof(pos_set_local));
    taskEXIT_CRITICAL(&mux1);

   // Read all encoder positions in one critical section
    taskENTER_CRITICAL(&mux);
    for (int i = 0; i < NUM_MOTORS; i++) {
      current_pos[i] = (float)Data_Buff[i] / 28.0f; //convert to no.of motror shaft rotations
    }
    taskEXIT_CRITICAL(&mux);

   for(int i=0;i<(NUM_MOTORS);i++){
     //calculate the setpoint in terms of motor shaft revolution
	   motor_pid[i].setPoint = pos_set_local[i]*GEAR_RATIO;
     
	   float control_signal = compute_pid(&motor_pid[i],current_pos[i]);
     
     // Map duty cycle
     float duty = fabs(control_signal) * map_val;
     duty = (duty > BIT_VALUE) ? BIT_VALUE : duty;

     // Determine direction (sign of control_signal)
     bool dir = (control_signal >= 0);

     // Update shared motor commands
     portENTER_CRITICAL(&pwm_mux);
     motor_commands[i].duty = (uint32_t)duty;
     motor_commands[i].direction = dir;
     portEXIT_CRITICAL(&pwm_mux); 

     ESP_LOGI(PWM_TAG, "control_signal %f", control_signal);
    }
   vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void pwm_update_Task(void *pvParam) {
  while (1) {
    portENTER_CRITICAL(&pwm_mux);
    for (int i = 0; i < NUM_MOTORS; i++) {
      // Set duty for Channel A (forward) or Channel B (reverse)
      if (motor_commands[i].direction) {
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, LEDC_CHANNEL[i], motor_commands[i].duty));
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, LEDC_CHANNEL[i]));
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, LEDC_CHANNEL[i], 0));  // Turn off reverse
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, LEDC_CHANNEL[i]));
      } 
      else {
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, LEDC_CHANNEL[i], 0));  // Turn off forward
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, LEDC_CHANNEL[i]));
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, LEDC_CHANNEL[i], motor_commands[i].duty));
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, LEDC_CHANNEL[i]));  
      }
    }
    portEXIT_CRITICAL(&pwm_mux);
    vTaskDelay(pdMS_TO_TICKS(1));  // Adjust frequency as needed
  }
}


void app_main(void){
  pcnt_app_main();
  BLE_app_main();
  pwm_app_main();

  for(int i = 0;i < NUM_MOTORS;i++){
  	motor_pid[i].kp=1.0;
 		motor_pid[i].ki=0.01;
 		motor_pid[i].kd=0.1;
 		motor_pid[i].integral =0.0;
 		motor_pid[i].prev_time = esp_timer_get_time();
 		motor_pid[i].setPoint=0.0;
 		motor_pid[i].prev_err=pos_set[i];
  }

 xTaskCreatePinnedToCore( compute_pid_Task,
                          "pid_task",
                          2048,
                          NULL,
                          (24|portPRIVILEGE_BIT),
                          &PID_taskHandle,
                          0
                          );
 xTaskCreatePinnedToCore( pwm_update_Task,
                          "pwm_task",
                          2048,
                          NULL,
                          (23|portPRIVILEGE_BIT),
                          &pwm_taskHandle,
                          1
                          );

}
