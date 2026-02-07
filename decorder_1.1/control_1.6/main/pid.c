#include "pid.h"
#include "pcnt.h"
#include "cpg.h"
#include "UART.h"

#define TAG "PID"
#define PID_QUEUE_LENGTH 1

// ====================== Globals ======================
static Motor motors[NUM_MOTORS];
QueueHandle_t motor_command_queue;
TaskHandle_t pid_loop_task = NULL;

// ====================== SAFE DOUBLE BUFFER ======================
volatile DRAM_ATTR float motor_cmd_buffer[2][NUM_MOTORS];
volatile DRAM_ATTR uint8_t current_write_index = 0;  // PID reads from this
portMUX_TYPE pid_buffer_swap_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t prev_read_index=0;
// Forward declarations
void drive_motor(Motor* m);
void stop_motor(Motor *m);

void update_i2c_motor_commands(float *new_commands) {
    uint8_t write_buffer = 1 - current_write_index;  // Write to opposite buffer
    
    // Copy to back buffer
    memcpy((void*)motor_cmd_buffer[write_buffer], new_commands, sizeof(float) * NUM_MOTORS);

    // Atomic swap - ensures PID sees consistent snapshot
    taskENTER_CRITICAL(&pid_buffer_swap_mux);
    current_write_index = write_buffer;
    taskEXIT_CRITICAL(&pid_buffer_swap_mux);
}

// PID reads latest available (non-blocking)
bool read_latest_motor_commands(motor_Command_t *dest){
    static uint8_t prev_buffer = 0;  // Track last read
    uint8_t read_buffer;
    
    taskENTER_CRITICAL(&cpg_ring_mux);
    read_buffer = write_index;
    taskEXIT_CRITICAL(&cpg_ring_mux);
    
    if (read_buffer == prev_buffer) {
        return false;  // No new data
    }

    // Copy from cmd_buffer
    memcpy(dest,(void*) cmd_buffer[read_buffer],sizeof(motor_Command_t) * NUM_MOTORS);

    prev_buffer = read_buffer;
    return true;
}

// ====================== PID Functions ======================
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->output = 0.0f;
}

void update_PID_gain(void){
    if (xSemaphoreTake(uart_mutex,0) == pdTRUE){
    for(int i=0;i<NUM_MOTORS;i++){
     //motors[i].pos_pid.Kp = update_motors[i].pos_pid.Kp;
     motors[i].pos_pid.Ki = update_motors[i].pos_pid.Ki;
     motors[i].pos_pid.Kd = update_motors[i].pos_pid.Kd;
    }
    xSemaphoreGive(uart_mutex);
    }
}

void PIDController_Update(PIDController *pid, float setpoint, int measurement, uint8_t i) {
    
    pid->prevError = pid->error;
    pid->lastOutput = pid->output;
    pid->prevMeasurement = (float)measurement;
  
    pid->error = setpoint - (float)measurement;

    //ESP_LOGE(TAG, "CPG[%1d] (out: %.2f)",i ,pid->error);

    // Proportional term
    pid->proportional = pid->Kp * pid->error;
    //derivative term
    pid->differentiator = pid->Kd*(pid->error - pid->prevError)/pid->T;
    //integral term
    pid->integrator += pid->Ki *pid->error;

    // Clamp integrator
    if (pid->integrator > pid->limMaxInt){
        pid->integrator = pid->limMaxInt;
    }
    if (pid->integrator < pid->limMinInt){
        pid->integrator = pid->limMinInt;
    }

    if(fabs(pid->error)<=STOP_THRESHOLD*10){
      pid->integrator *= 0.9f; // decay
    }

    //calculate PID output 
    pid->output = pid->proportional +pid->differentiator+pid->integrator;
    //normalize output for stability
    pid->output = 0.7 * pid->output + 0.3 * pid->lastOutput;

    //clamp PID output 

    if (pid->output > pid->limMax){
        pid->output = pid->limMax;
    }
    if (pid->output < pid->limMin){
        pid->output = pid->limMin;
    }

    //ESP_LOGE(TAG, "CPG[%1d] (out: %.2f)",i ,pid->output)


}

// ====================== Position Loop ======================
void position_loop_task(void *arg) {
    
    uint8_t stall_count[NUM_MOTORS] = {0};
    static motor_Command_t cpg[NUM_MOTORS];
    int current_encoders[NUM_MOTORS] = {0};  // This packet will hold our local, consistent copy of all encoder value

    static uint32_t cpg_error_count = 0;
    float pidOutput[NUM_MOTORS];
    static uint32_t log_counter = 0;

    while (1) {

        update_PID_gain();

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint64_t start = esp_timer_get_time();
        
        if (read_latest_motor_commands(cpg)==true){
            for(int i =0 ; i<NUM_MOTORS;i++){
             motors[i].target_position = cpg[i].motor_target;
             motors[i].active = cpg[i].activate;
            }
        } else{
            // Use existing data, don't log error every time
            cpg_error_count++;
            if (cpg_error_count % 10 == 0) {  // Only log occasionally
                ESP_LOGW(TAG, "Receiving cpg values failed No new data: (count: %lu)", cpg_error_count);
                cpg_error_count=0;
            }
        }

        // Throttled log (every ~100 loops ~1s)
        log_counter++;
        if (log_counter % 100 == 0) {
            printf("CPG Target[0]: %.2f\n", motors[0].target_position);
            fflush(stdout);
        }
        

        // At the start of the loop, get the MOST RECENT encoder snapshot
        for (int i = 0; i < NUM_ENCODERS; i++){
         ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit[i],&current_encoders[i]));
         //ESP_LOGE(TAG, "PCNT[%1d] (count: %1d)",i ,current_encoders[i]);

        }
        
        for (int i = 0; i < NUM_MOTORS; i++) {

            motors[i].current_position = current_encoders[i];

            //ESP_LOGE(TAG, "CPG[%1d] (count: %.2f)",i ,motors[i].target_position);

            PIDController_Update(&motors[i].pos_pid,motors[i].target_position,
                                 motors[i].current_position, i);

            motors[i].last_enc_val = motors[i].current_position;

            if (!motors[i].active) {
               if(motors[i].current_position<=STOP_THRESHOLD*5){
                  motors[i].pos_pid.output = 0;
                  continue;
                }
            }

            float abs_error = fabs(motors[i].pos_pid.error);
            if (abs_error > TOL) {
                if (motors[i].current_position == motors[i].last_enc_val) {
                    stall_count[i]++;
                    if (stall_count[i] >= 5) {
                        //motors[i].pos_pid.output = PWM_MAX;
                    }
                } else {
                    stall_count[i] = 0;
                }
            }

        }

        for(int i =0;i<NUM_MOTORS;i++){
          pidOutput[i] = motors[i].pos_pid.output;
        }

        // send pid output to the i2c
        update_i2c_motor_commands(pidOutput);

        /*for(int i=0;i<NUM_MOTORS;i++){
         ESP_LOGE(TAG, "PID OUT s[%1d] : %.2f",i ,motors[i].pos_pid.output);
        }*/
        // Send notification to i2c task
        xTaskNotifyGive(i2c_send_task);
    
        xQueueOverwrite(encorderQue,current_encoders);

        // PID work
        uint64_t elapsed = esp_timer_get_time() - start;
        if (elapsed > 200) ESP_LOGW(TAG, "PID loop took %llu µs", elapsed);
    }
}

// ====================== Motor Setup ======================
void pid_app_main(void) {
    ESP_LOGI(TAG, "Starting PID application...");
    
    motor_command_queue = xQueueCreate(PID_QUEUE_LENGTH, sizeof(motor_Command_t)*NUM_MOTORS);
    if (!motor_command_queue) {
        ESP_LOGE(TAG, "Creating motor command queue failed");
        return;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i + 1;
        motors[i].target_position = 0.0f;
        motors[i].current_position = 0;
        motors[i].active = false;

        motors[i].pos_pid.Kp = 1.00f;
        motors[i].pos_pid.Ki = 0.00f;
        motors[i].pos_pid.Kd = 0.00f;

        motors[i].pos_pid.T = PID_DT;
        motors[i].pos_pid.limMin = -PWM_MAX;
        motors[i].pos_pid.limMax = PWM_MAX;
        motors[i].pos_pid.limMinInt = -400.0f;
        motors[i].pos_pid.limMaxInt = 400.0f;

        PIDController_Init(&motors[i].pos_pid);
    }

    xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 5120, NULL, 20, &pid_loop_task,1);

    ESP_LOGI(TAG, "PID application started successfully");
}  