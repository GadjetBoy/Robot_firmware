#include "robot_controller.h"

portMUX_TYPE pid_mux = portMUX_INITIALIZER_UNLOCKED;

//global variables to store BLE recived values
float pos_target_val[NUM_MOTORS]; 
float neg_target_val[NUM_MOTORS];
volatile uint8_t control;
float map_val;
float motor_pos[NUM_MOTORS];


//declare task handle
TaskHandle_t PID_taskHandle = NULL;
TaskHandle_t pwm_taskHandle = NULL;

// PID Structure
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float prev_time;
} PIDController;

// Motor Control Structure
typedef struct {
    int32_t current_position;
    float target_position;
    PIDController pid;
    uint8_t pwm_channel;
    uint8_t direction;
    uint32_t pwm;
    uint8_t m_num;
} Motor;

Motor m1,m2,m3,m4;
static uint8_t last_control;

// State Machine States
typedef enum {
    STATE_IDLE = 0,
    // Individual states
    STATE_MOT1_POS = 1 << 0,
    STATE_MOT2_POS = 1 << 1,
    STATE_MOT3_POS = 1 << 2,
    STATE_MOT4_POS = 1 << 3,
    STATE_MOT1_NEG = 1 << 4,
    STATE_MOT2_NEG = 1 << 5,
    STATE_MOT3_NEG = 1 << 6,
    STATE_MOT4_NEG = 1 << 7,
} SystemState;

struct {
 uint8_t MOT1_POS_STATE:1;
 uint8_t MOT2_POS_STATE:1;
 uint8_t MOT3_POS_STATE:1;
 uint8_t MOT4_POS_STATE:1;

 uint8_t MOT1_NEG_STATE:1;
 uint8_t MOT2_NEG_STATE:1;
 uint8_t MOT3_NEG_STATE:1;
 uint8_t MOT4_NEG_STATE:1;

 uint8_t M_STATE_IDLE  :1;

}MotorState;

SystemState current_state;
// PID Initialization
void init_pid(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/***************************************************PID Calculation*********************************************/

float compute_pid(PIDController *pid, float error,uint8_t i){
 //calculate the elapsed time
 float current_t = esp_timer_get_time();
 //calculate dt value
 portENTER_CRITICAL(&pid_mux);
 float dt = (current_t-(pid->prev_time))/1e6f;
 if(dt<=0){    
   dt = 1e-6f;
  } 
 //derivative
 float derivative = (error-(pid->prev_error))/dt;
 
 //integral
 (pid->integral) += (error * dt); 
 
 // Anti-windup clamping
 if ((pid->integral) > MAX_INTEGRAL) {
    pid->integral = MAX_INTEGRAL;
  }
 else if ((pid->integral) < -MAX_INTEGRAL) {
    pid->integral = -MAX_INTEGRAL;
  }
  if(error <= TOLERANCE){
    pid->integral = 0;
  }


  //calculate control signal value
 float U = ((pid->kp) * error) + ((pid->kd) * derivative )+ ((pid->ki) * (pid->integral));
 portEXIT_CRITICAL(&pid_mux);

 //store values 
 pid->prev_error = error;
 pid->prev_time = current_t;
 ESP_LOGI(PID_TAG, "control_signal[%d]= %.2f",i,U);
 return U ;
}

/**************************************************Motor Control Update*********************************/
void update_motor(Motor *m) {

    //ESP_LOGI(PID_TAG, "pcnt_count_begin[%d]= %.2f",m->m_num,((float)pcnt_count[m->m_num-1]));
    //ESP_LOGI(PID_TAG, "pcnt_count_at_task_upadte[%d]= %.2f",m->m_num,(m->current_position)*EN_PPR);

     float error = m->target_position - m->current_position;
    
     // Determine direction and PWM value
     //m->direction = (output > 0) ? 1 : -1;
    
     //if(control == 1){
           // Calculate PID output
           //portENTER_CRITICAL(&pid_mux);
           float output = compute_pid(&m->pid, error,m->m_num);
           //portEXIT_CRITICAL(&pid_mux);

          if(output<0){
              m->direction = -1;
              ESP_LOGI(PID_TAG, " LEDC_HIGH_SPEED_MODE drive motor[%d] reverse\n",m->m_num);  
            }
         else{
              m->direction = 1;
              ESP_LOGI(PID_TAG, " LEDC_LOW_SPEED_MODE drive motor[%d] forward\n",m->m_num);  
            }

            float pwm = fabs(output);//*map_val
            
            if(pwm>BIT_VALUE){
              m->pwm = (BIT_VALUE -1);  
            }
            else if(pwm<100){
                m->pwm =100 ;
            }
            else{
                m->pwm = pwm;
            }
            
       // }
    
     /*else{
         if(error<0){
             m->direction = -1;
             ESP_LOGI(PID_TAG, " 0x%x drive motors", LEDC_MODEB);  
            }
         else{
             m->direction = 1;
             ESP_LOGI(PID_TAG, " 0x%x drive motors", LEDC_MODEA);  
            } 
          m->pwm = (BIT_VALUE -1);
        }*/
     ESP_LOGI(PID_TAG, "PWM_signal[%d]= %.f",m->m_num,(float)m->pwm);

     ESP_LOGI(PID_TAG, "target_POS[%d]= %.2f",m->m_num,m->target_position);

    ESP_LOGI(PID_TAG, "pcnt_count at pid end[%d]= %.2f",m->m_num,((float)pcnt_count[m->m_num-1]));
    // Set PWM and direction pins
    drive_motors(m->pwm_channel, m->pwm,m->direction);
    
    
}

void drive_motors(uint8_t pwm_channel,uint32_t pwm,int8_t direction){
  portENTER_CRITICAL(&pid_mux);
  // Set duty for Channel A (forward) or Channel B (reverse)
      if (direction==1) {
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, LEDC_A_CHANNEL[pwm_channel], pwm));
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, LEDC_A_CHANNEL[pwm_channel]));
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, LEDC_B_CHANNEL[pwm_channel],0));  // Turn off reverse
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, LEDC_B_CHANNEL[pwm_channel]));

      } 
      else if(direction==-1){
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, LEDC_A_CHANNEL[pwm_channel],0));  // Turn off forward
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, LEDC_A_CHANNEL[pwm_channel]));
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, LEDC_B_CHANNEL[pwm_channel], pwm));
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, LEDC_B_CHANNEL[pwm_channel]));
           
      }
        
  portEXIT_CRITICAL(&pid_mux);
}
void stop_motor(uint8_t pwm_channel){

         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEA, LEDC_A_CHANNEL[pwm_channel],0));  // Turn off 
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEA, LEDC_A_CHANNEL[pwm_channel]));
         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODEB, LEDC_B_CHANNEL[pwm_channel],0));
         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODEB, LEDC_B_CHANNEL[pwm_channel]));  
      
}
/*********************************************************** State Machine Handler********************************************/
void handle_state_machine() { 
    switch(current_state) {
 
        case STATE_MOT1_POS:  
            if(fabs(m1.current_position - pos_target_val[0]) < TOLERANCE /*&& 
               fabs(m3.current_position - pos_target_val[2]) < TOLERANCE*/) {    
                stop_motor(m1.pwm_channel);
                stop_motor(m3.pwm_channel);
                current_state = STATE_MOT2_POS;
            }
            break;
            
        case STATE_MOT2_POS:  
            if(fabs(m2.current_position - pos_target_val[1]) < TOLERANCE /*&& 
               fabs(m4.current_position - pos_target_val[3]) < TOLERANCE*/) {
                stop_motor(m2.pwm_channel);
                stop_motor(m4.pwm_channel);
                current_state = STATE_MOT1_NEG;
            } 
            break;
            
        case STATE_MOT1_NEG:  
            if(fabs(m1.current_position - neg_target_val[0]) < TOLERANCE /*&&  
               fabs(m3.current_position - neg_target_val[2]) < TOLERANCE*/) {
                stop_motor(m1.pwm_channel);
                stop_motor(m3.pwm_channel);
                current_state = STATE_MOT2_NEG; 
            }
            break;
            
        case STATE_MOT2_NEG:  
            if(fabs(m2.current_position - neg_target_val[1]) <TOLERANCE /*&& 
               fabs(m4.current_position - neg_target_val[3]) < TOLERANCE*/) {
               stop_motor(m2.pwm_channel);
               stop_motor(m4.pwm_channel);
               current_state = STATE_MOT1_POS; 
            }
            break;
        case STATE_IDLE:
                  ESP_LOGI(PID_TAG, "state IDLE state machine: 0x%x", current_state); 

            break;
        
        
        default:
              ESP_LOGE(PID_TAG, "Invalid state for handle stae machine: 0x%x", current_state);
            break;
    }
}


void mode_set(void){
    // Validate control value first

    if(control!=last_control){

     switch(control){
          case 1:
             current_state = STATE_MOT1_POS;
             MotorState.MOT1_POS_STATE = 1;
          break;
          case 0:
             current_state = STATE_IDLE;
             MotorState.M_STATE_IDLE = 1;
          break;
          case 2:
             current_state = STATE_MOT2_POS; 
             MotorState.MOT2_POS_STATE = 1;
          break;
          default:
             ESP_LOGI(PID_TAG, "no valid mode avilable");
             current_state = STATE_IDLE;
          break;
       }
        last_control=control;
   }
  ESP_LOGI(PID_TAG, "control= %d",control);
}
void update_pos(void){
    int32_t temp_motor_pos_buf[NUM_MOTORS]; 
     taskENTER_CRITICAL(&pcnt_mux);
     memcpy(temp_motor_pos_buf,(void *)pcnt_count,sizeof(int32_t)*NUM_MOTORS);
     taskEXIT_CRITICAL(&pcnt_mux);

    m1.current_position = (float)temp_motor_pos_buf[0]/EN_PPR;
    m2.current_position = (float)temp_motor_pos_buf[1]/EN_PPR;
    m3.current_position = (float)temp_motor_pos_buf[2]/EN_PPR;
    m4.current_position = (float)temp_motor_pos_buf[3]/EN_PPR;

}

void pid_task(void *pvParam){
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000/PID_UPDATE_FREQ);

    m1.pid.prev_time = esp_timer_get_time();
    m2.pid.prev_time = esp_timer_get_time();
    m3.pid.prev_time = esp_timer_get_time();
    m4.pid.prev_time = esp_timer_get_time();

    while(1) {

        //update target positions
        taskENTER_CRITICAL(&ble_mux);
        memcpy(pos_target_val,(void *)float_val,sizeof(float)*4);
        memcpy(neg_target_val,((void *)float_val)+(4*sizeof(float)),sizeof(float)*4);
        control = int_val[0];
        taskEXIT_CRITICAL(&ble_mux);

       /* for(int i=0; i<4;i++){
            ESP_LOGI(PWM_TAG, "neg_target_val[%d]= %.2f",i,neg_target_val[i]);
        }*/

       for(int i =0;i<NUM_MOTORS;i++){
           pos_target_val[i]=pos_target_val[i]*GEAR_RATIO;
           neg_target_val[i]=neg_target_val[i]*GEAR_RATIO;   
        }

        mode_set();
        update_pos();
        handle_state_machine();
        
        // Set target positions based on state
        switch(current_state) {
            case STATE_MOT1_POS:
                  m1.target_position = pos_target_val[0];
                  m3.target_position = pos_target_val[2];
                  // Update motor controls
                  update_motor(&m1);
                  update_motor(&m3);
                break;
            case STATE_MOT2_POS:
                  m2.target_position = pos_target_val[1];
                  m4.target_position = pos_target_val[3];
                  // Update motor controls
                  update_motor(&m2);
                  update_motor(&m4);
                break;
            case STATE_MOT1_NEG:
                  m1.target_position = neg_target_val[0];
                  m3.target_position = neg_target_val[2];
                  // Update motor controls
                  update_motor(&m1);
                  update_motor(&m3);
                break;
            case STATE_MOT2_NEG:
                  m2.target_position = neg_target_val[1];
                  m4.target_position = neg_target_val[3];
                  // Update motor controls
                  update_motor(&m2);
                  update_motor(&m4);
                break;
            case STATE_IDLE:
                if((fabs(m1.current_position - IDLE_STATE_VAL)<TOLERANCE) && (fabs(m2.current_position - IDLE_STATE_VAL)<TOLERANCE) /*&&
                  (fabs(m3.current_position - IDLE_STATE_VAL)<TOLERANCE) && (fabs(m4.current_position - IDLE_STATE_VAL)<TOLERANCE)*/){
                      stop_motor(m1.pwm_channel);
                      stop_motor(m2.pwm_channel);
                      stop_motor(m3.pwm_channel);
                      stop_motor(m4.pwm_channel);
                    }
                 else{
                      m1.target_position = IDLE_STATE_VAL;
                      m2.target_position = IDLE_STATE_VAL;
                      m3.target_position = IDLE_STATE_VAL;
                      m4.target_position = IDLE_STATE_VAL;
                      update_motor(&m1);
                      update_motor(&m2);
                      update_motor(&m3);
                      update_motor(&m4);
                    }
                break;
            default:
              ESP_LOGE(PID_TAG, "Invalid state: 0x%x", current_state);
            
            break;
        }

        //esp_task_wdt_reset();

        xTaskDelayUntil( &xLastWakeTime, xFrequency);
        
    }
    

}

void pid_app_main(void){

    // Motor 1 Initialization
    m1.m_num = 1;
    m1.target_position = 0;
    init_pid(&m1.pid,(2.00), (0.9), (1.0));  // Tune these values
    m1.pwm_channel = 0;
    
    // Motor 2 Initialization
    m2.m_num = 2;
    m2.target_position = 0;
    init_pid(&m2.pid,(2.00), (0.9), (1.0));  // Tune these values
    m2.pwm_channel = 1;
    

    // Motor 3 Initialization
    m3.m_num = 3;
    m3.target_position = 0;
    init_pid(&m3.pid,(2.00), (0.9), (1.0));  // Tune these values
    m3.pwm_channel = 2;
    
    // Motor 4 Initialization
    m4.m_num = 4;
    m4.target_position = 0;
    init_pid(&m4.pid,(2.00), (0.9), (1.0));  // Tune these values
    m4.pwm_channel = 3;
    
    
      MotorState.MOT1_POS_STATE = 0,
      MotorState.MOT2_POS_STATE = 0,
      MotorState.MOT3_POS_STATE = 0,
      MotorState.MOT4_POS_STATE = 0,
      MotorState.MOT1_NEG_STATE = 0,
      MotorState.MOT2_NEG_STATE = 0,
      MotorState.MOT3_NEG_STATE = 0,
      MotorState.MOT4_NEG_STATE = 0,
      MotorState.M_STATE_IDLE   = 0;

 

    //map_val = BIT_VALUE / MAX_CONTROL_SIGNAL; // Precompute outside loop

    xTaskCreatePinnedToCore(pid_task, 
                           "PID_task", 
                            4096, 
                            NULL, 
                            (23|portPRIVILEGE_BIT),
                            &PID_taskHandle, 
                           1);

}
