#include "pcnt.h"
#include "cpg.h"
#include "uart.h"
#include "ble.h"
#include "gate.h"

//============================================================PID==================================================================
#define PID_QUEUE_LENGTH 1

// Math Constants
#define TWO_PI (2.0f * 3.1415926535f)

// Tags for logging (consistent)
#define TAG_PID "PID_CTRL"
#define TAG_CPG "CPG_NET"
#define TAG_MOTOR "MOTOR_CMD"
#define TAG_SEQ "SEQ_RUN"
#define TAG_UART "UART_SEND"

// Debug logging levels (easy to toggle)
#define ENABLE_PID_DEBUG 0
#define ENABLE_CPG_DEBUG 0
#define ENABLE_MOTOR_DEBUG 0
#define ENABLE_UART_DEBUG 0
   
// ====================== Globals ======================
// Use non-static for extern compatibility
DRAM_ATTR Motor motors[NUM_MOTORS];
volatile DRAM_ATTR Motor Update_PID; 

volatile DRAM_ATTR float motor_cmd_buffer[2][NUM_MOTORS];
volatile DRAM_ATTR uint8_t current_write_index = 0;
portMUX_TYPE pid_buffer_swap_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE cpg_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint8_t last_cmd;

// CPG globals
volatile Oscillator cpg_network[NUM_OSCILLATORS];
volatile osc_pram CPG_network_pram;
volatile uint8_t cmd = CPG_MODE_STANDBY;
volatile CPG_RunMode cpg_run_mode = CPG_MODE_IDLE;
volatile body_posture_t posture = BODY_POSTURE_NORMAL;
turning_modes turn_mode = MODE_NORMAL;
volatile float coupling_weights[NUM_OSCILLATORS][NUM_OSCILLATORS];
volatile float phase_offsets[NUM_OSCILLATORS][NUM_OSCILLATORS];
volatile LegOrientation leg_orientation = LEG_ORIENTATION_NORMAL;
SemaphoreHandle_t cpg_params_mutex;
volatile bool pivot_turn =false;

static esp_timer_handle_t cpg_timer_handle;
static TaskHandle_t cpg_task_handle = NULL;
static TaskHandle_t seq_task_handle = NULL;

static uint64_t start;

//volatile SequenceMode pending_mode = MODE_IDLE;
//volatile bool mode_change_pending = false;

// ====================== Utility Functions ======================
inline void update_uart_motor_commands(float *new_commands) {
    uint8_t write_buffer = 1 - current_write_index; // Write to opposite buffer
   
    // Copy to back buffer
    memcpy((void*)motor_cmd_buffer[write_buffer], new_commands, sizeof(float) * NUM_MOTORS);
    // Atomic swap - ensures PID sees consistent snapshot
    taskENTER_CRITICAL(&pid_buffer_swap_mux);
    current_write_index = write_buffer;
    taskEXIT_CRITICAL(&pid_buffer_swap_mux);
}

void update_orientation_from_ble(uint8_t cmd)
{
    if (cmd == 6)
        leg_orientation = LEG_ORIENTATION_NORMAL;
    else if (cmd == 7)
        leg_orientation = LEG_ORIENTATION_INVERTED;
}

static inline uint8_t get_commands(void) {

    taskENTER_CRITICAL(&ble_mux);
    cmd = byte_val[0];
    taskEXIT_CRITICAL(&ble_mux);

    update_orientation_from_ble(cmd);

    /*if(cmd == MODE_PIVOT_TURN){
        pivot_turn = true;
    }
    if(){
       pivot_turn = false; 
    }*/

    return cmd;
}

// ====================== PID Functions ======================
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->output = 0.0f;
}

// Dynamic gain update based on frequency (called periodically)
void update_PID_gain(void) {

    //if (xSemaphoreTake(pid_params_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
    taskENTER_CRITICAL(&ble_mux);
       Update_PID.pos_pid.Kp = pid_ble_params.Kp;
       Update_PID.pos_pid.Ki = pid_ble_params.Ki;
       Update_PID.pos_pid.Kd = pid_ble_params.Kd;
    taskEXIT_CRITICAL(&ble_mux);
        //xSemaphoreGive(pid_params_mutex);
    //}
    
    float freq_scale = fminf(CPG_frequency / 0.1f, 2.5f);  // FIXED: Cap at 2.5x (less aggressive than 3.0)
    for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].pos_pid.Kp = Update_PID.pos_pid.Kp * fminf(1.0f + 0.3f * freq_scale, 2.5f);  // Cap eff Kp<=5
            motors[i].pos_pid.Kd = Update_PID.pos_pid.Kd * (1.0f + 0.6f * freq_scale);  // Milder Kd scale
            motors[i].pos_pid.Ki = fmaxf(Update_PID.pos_pid.Ki * 0.8f, 0.005f);  // FIXED: Floor at 0.005, less reduction
            motors[i].pos_pid.limMaxInt = 300.0f / freq_scale;  // FIXED: Tighter (200→150) to limit windup             
    }
}


void PIDController_Update(PIDController *pid, float setpoint, int measurement, uint8_t i) {
   
    pid->prevError = pid->error;
    pid->lastOutput = pid->output;
    pid->prevMeasurement = (float)measurement;
 
    pid->error = setpoint - (float)measurement;
    //ESP_LOGE(TAG, "CPG[%1d] (out: %.2f)",i ,pid->error);

    float abs_error = fabsf(pid->error);
    float abs_output = fabsf(pid->lastOutput);
    float kp_scale = 1.0f;

    if (abs_error < STOP_THRESHOLD * 3 || abs_output > PWM_MAX * 0.6f) {  // Near zero or saturated (peak)
      kp_scale = 0.5f + 0.5f * fminf(abs_error / (STOP_THRESHOLD * 3), abs_output / PWM_MAX); // FIXED: Blend error/output for peak focus
    }
    pid->proportional = pid->Kp * kp_scale * pid->error;

    // Derivative (add output blend if needed)
    float kd_scale = 1.0f;
    if (abs_output > PWM_MAX * 0.7f) kd_scale = 1.8f;  // FIXED: Higher (1.5→1.8) for low Kd
    pid->differentiator = pid->Kd * kd_scale * (pid->error - pid->prevError) / pid->T;

    //integral term
    pid->integrator += pid->Ki *pid->error*pid->T;
    // Clamp integrator
    if (pid->integrator > pid->limMaxInt){
        pid->integrator = pid->limMaxInt;
    }
    if (pid->integrator < pid->limMinInt){
        pid->integrator = pid->limMinInt;
    }
    if(fabs(pid->error)<=STOP_THRESHOLD*5){
      pid->integrator *= 0.9f; // decay
    }
    //calculate PID output
    pid->output = pid->proportional +pid->differentiator+pid->integrator;
    
    float raw_output = pid->proportional + pid->differentiator + pid->integrator;

    // Anti-windup: Back-calculate integrator if saturated
    if (raw_output > pid->limMax) {
     pid->integrator -= (raw_output - pid->limMax) * pid->Ki * pid->T; // Undo excess
     pid->output = pid->limMax;
    } else if (raw_output < pid->limMin) {
     pid->integrator -= (raw_output - pid->limMin) * pid->Ki * pid->T;
     pid->output = pid->limMin;
    } else {
     pid->output = raw_output;
    }

    pid->output = 0.5f * pid->output + 0.5f * pid->lastOutput;
    
    
}

// ====================== Position Loop ======================
void run_position_loop(){
    static int current_encoders[NUM_MOTORS] = {0}; // Local snapshot
    static int filtered_enc[NUM_MOTORS] = {0}; // IIR filter state
    static uint32_t log_counter = 0;
    log_counter++;
    
    // Update gains periodically (throttled)
    if (log_counter % 5 == 0) { // ~0.1s at 2500Hz
        update_PID_gain();
        log_counter = 0;
    }
    // Read encoders (snapshot)
    for (int i = 0; i < NUM_ENCODERS; i++) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit[i], &current_encoders[i]));
    }
    // Filter and update positions
    for (int i = 0; i < NUM_MOTORS; i++) {
        filtered_enc[i] = (int)(0.2f * filtered_enc[i] + 0.8f * current_encoders[i]);
        motors[i].current_position = filtered_enc[i];
    }
    // Run PID for each motor and collect outputs
    float pid_outputs[NUM_MOTORS];

    for (int i = 0; i < NUM_MOTORS; i++) {
        PIDController_Update(&motors[i].pos_pid, motors[i].target_position,
                             motors[i].current_position, (uint8_t)i);

        pid_outputs[i] = motors[i].pos_pid.output; // Collect for UART

        if (!motors[i].active && cpg_run_mode == CPG_MODE_IDLE){
            if (fabsf((float)motors[i].current_position) <= STOP_THRESHOLD*2){
                pid_outputs[i] = 0.0f;
            }
        }

       
        motors[i].last_enc_val = motors[i].current_position;
    }
    // Send PID outputs to UART via double buffer
    update_uart_motor_commands(pid_outputs);

    xTaskNotifyGive(uart_send_task);
       
    xQueueOverwrite(encorderQue,filtered_enc);
    
}

// ====================== Motor Setup ======================
void pid_app_main(void) {
    ESP_LOGI(TAG_PID, "Starting PID application...");
   
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].m_num = i + 1;
        motors[i].target_position = 0.0f;
        motors[i].current_position = 0;
        motors[i].active = false;
        motors[i].pos_pid.Kp = 2.00f;
        motors[i].pos_pid.Ki = 0.00f;
        motors[i].pos_pid.Kd = 0.00f;
        motors[i].pos_pid.T = PID_DT;
        motors[i].pos_pid.limMin = -PWM_MAX;
        motors[i].pos_pid.limMax = PWM_MAX;
        motors[i].pos_pid.limMinInt = -500.0f;
        motors[i].pos_pid.limMaxInt = 500.0f;
        PIDController_Init(&motors[i].pos_pid);
    }
    //xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 5120, NULL, 20, &pid_loop_task,1);
    ESP_LOGI(TAG_PID, "PID application started successfully");
}

// ====================== CPG Functions ======================

// Timer ISR (notifies task)
void IRAM_ATTR cpg_timer_callback(void* arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(cpg_task_handle, &woken);
    if (woken) portYIELD_FROM_ISR();
}

// Set motor target with feedforward velocity term
void motor_set(uint8_t i, float target, bool set) {
    if (i >= NUM_MOTORS) return;

    float signed_target =target;

    //Sign flip for right legs (FR/BR: indices 1,3,5,7) for symmetry (forward gait)
    bool is_right_leg = (i == FRH || i == BRH);  // Adjust indices if mapping differs
    if(is_right_leg ){
      signed_target = -signed_target ;  // Mirror right for opposite swing
    }
    
    bool is_knee = (i == FLK || i == FRK || i == BLK || i == BRK);// Check if it's a knee joint
    if (is_knee && (last_cmd == MODE_CREEP || last_cmd == MODE_TURTLE)) {

         float knee_offset = cpg_network[i].offset;

         // Prevent backward pull
         if (signed_target < knee_offset) {
             signed_target = knee_offset;
            }
    }

    if ( leg_orientation == LEG_ORIENTATION_INVERTED) {
     signed_target = -signed_target;
    }

    // Velocity FF (on signed target for correct direction)
    float duty = CPG_network_pram.duty_cycle;
    float phase = cpg_network[i].phase;
    float alpha = (phase < M_PI) ? (1.0f / (2.0f * (1.0f - duty))) : (1.0f / (2.0f * duty));

    // Correct velocity of a warped sine: v = amp * (omega * alpha) * cos(phase)
    float vel_ff = cpg_network[i].amplitude * (cpg_network[i].omega * alpha) * cosf(phase);
    
    motors[i].target_position = signed_target + (vel_ff * PID_DT * 0.6f);

    if(!set){
      motors[i].target_position = 0;  
    }

    motors[i].active = set;
    motors[i].prev_target = signed_target;

    if (ENABLE_MOTOR_DEBUG) {
        ESP_LOGI(TAG_MOTOR, "Motor %d target=%.2f (active=%s)", i, target, set ? "true" : "false");
    }
}

void cpg_task(void *pvParameters) {
    float d_phi[NUM_OSCILLATORS];
    float phase[NUM_OSCILLATORS];

    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        start = esp_timer_get_time();
        if (xSemaphoreTake(cpg_params_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGE(TAG_CPG, "CPG mutex timeout!");
            continue;
        }

        // 1. Run the smoothing (alpha = 0.05 is a good start)
        //smooth_gait_parameters(0.05f); 

        if (cpg_run_mode == CPG_MODE_IDLE) {   
            vTaskDelay(pdMS_TO_TICKS(2));
            for(int i = 0;i<NUM_MOTORS;i++){
             motor_set((uint8_t)i, 0,false);
            }
            xSemaphoreGive(cpg_params_mutex);
            run_position_loop();
            continue;
        }
        
        // 2. Update the CPG physics
        cpg_update(d_phi,phase); 

        xSemaphoreGive(cpg_params_mutex);

        // Trigger PID loop
        //ESP_LOGE(TAG_CPG, "cpg command position loop");
        run_position_loop();

        uint64_t elapsed = esp_timer_get_time() - start;
        if (elapsed > CPG_UPDATE_RATE_US && ENABLE_CPG_DEBUG) {
            ESP_LOGW(TAG_CPG, "CPG update overrun: %llu us", elapsed);
        }
        
    }
}

inline void cpg_update(float* d_phi, float* phase){

        // 1. Snapshot phases and compute coupling

        for (int i = 0; i < NUM_OSCILLATORS; i++) {
           phase[i] = cpg_network[i].phase;

          // --- NEW: Asymmetric Timing Calculation ---
          float duty = CPG_network_pram.duty_cycle; 
          if (duty < 0.1f) duty = 0.5f; // Safety floor
    
          float alpha; 
          // If phase is [0, PI], we are in Swing. If [PI, 2PI], we are in Stance.
          // We scale the frequency so the total period remains constant.
          if (phase[i] < M_PI) {
              // Swing Phase speed multiplier
              alpha = 1.0f / (2.0f * (1.0f - duty));
            } else {
                  // Stance Phase speed multiplier
                  alpha = 1.0f / (2.0f * duty);
            }

          float coupling_term = 0.0f;
          for (int j = 0; j < NUM_OSCILLATORS; j++) {
              if (i == j) continue;
              float sin_term = sinf(phase[j] - phase[i] - phase_offsets[i][j]);
              float norm_amp = cpg_network[i].amplitude / CPG_network_pram.max_amp;

               coupling_term += coupling_weights[i][j] * norm_amp * sin_term;
            }

          // Apply the alpha multiplier to the base frequency only
          float damping = 0.1f;
          float warped_omega = cpg_network[i].omega * alpha;
          d_phi[i] = (1.0f - damping) * warped_omega + damping * coupling_term;
        }

        // 2. Update phases and outputs
        for (int i = 0; i < NUM_OSCILLATORS; i++) {

            cpg_network[i].phase += d_phi[i] * CPG_DT;

            cpg_network[i].phase = fmodf(cpg_network[i].phase, TWO_PI);
            if (cpg_network[i].phase < 0.0f) cpg_network[i].phase += TWO_PI;

            // --- Amplitude stabilization ---
            float target_amp = (i % 2 == 0) ? CPG_network_pram.hip_amp:CPG_network_pram.knee_amp;
            if (fabsf(target_amp - cpg_network[i].amplitude) > 1.0f) {  // Only correct if drifted >1 count
              cpg_network[i].amplitude += 0.008f * (target_amp - cpg_network[i].amplitude)*CPG_DT; // smooth correction
            }

            // --- Bias correction (prevent drift) ---
            float target_offset = (i % 2 != 0) ? CPG_network_pram.knee_offset : CPG_network_pram.hip_offset;
            cpg_network[i].offset += 0.050f * (target_offset - cpg_network[i].offset) * CPG_DT;

            cpg_network[i].output = cpg_network[i].offset +
                                    cpg_network[i].amplitude * sinf(cpg_network[i].phase);

            //ESP_LOGW(TAG_CPG, "cpg update output");
            motor_set((uint8_t)i, cpg_network[i].output,
                      cpg_network[i].amplitude != 0.0f);

        }
}

void smooth_gait_parameters(float alpha) {
    // Smooth Amplitudes
    CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp_left + alpha * (CPG_network_pram.hip_amp_left_target - CPG_network_pram.hip_amp_left);
    CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp_right + alpha * (CPG_network_pram.hip_amp_right_target - CPG_network_pram.hip_amp_right);
    
    CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp_left + alpha * (CPG_network_pram.knee_amp_left_target - CPG_network_pram.knee_amp_left);
    CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp_right + alpha * (CPG_network_pram.knee_amp_right_target - CPG_network_pram.knee_amp_right);

    // Smooth Duty Cycle (This solves your transition issue!)
    CPG_network_pram.duty_cycle = CPG_network_pram.duty_cycle + alpha * (CPG_network_pram.duty_cycle_target - CPG_network_pram.duty_cycle);

    // Update the actual oscillators with the "smoothed" current values
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;

    // Apply to Left Hips
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    // Apply to Right Hips
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    
    // Apply to Left Knees
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    // Apply to Right Knees
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
}

// Set oscillator params (helper)
void set_oscillator_params(int i, float omega, float amp, float offset) {
    if (i < 0 || i >= NUM_OSCILLATORS) return;
    cpg_network[i].omega = omega;
    cpg_network[i].amplitude = amp;
    cpg_network[i].offset = offset;
    if (ENABLE_CPG_DEBUG) {
        ESP_LOGI(TAG_CPG, "Osc %d: omega=%.2f, amp=%.2f, offset=%.2f", i, omega, amp, offset);
    }
}

// Update base parameters (called when freq changes)
void Update_Oscillator_base_parameters(void) {
    CPG_network_pram.base_freq = TWO_PI * CPG_frequency;
    ESP_LOGI(TAG_CPG, "Base params updated for freq=%.2f Hz", CPG_frequency);
}

static inline void set_system_idle(){
     vTaskDelay(pdMS_TO_TICKS(10));
     set_gait_standby();
}

// Sequence runner (checks command changes)

void command_runner_task(void *arg) {
    uint8_t cmd;
    last_cmd = 255;
    while (1) {
        cmd = get_commands();
        if (cmd != last_cmd) {
            ESP_LOGE(TAG_SEQ, "Mode change: %d -> %d", last_cmd, cmd);
            // Take mutex, set gait, release mutex QUICKLY
            if (xSemaphoreTake(cpg_params_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                switch (cmd) {
                    case CPG_MODE_IDLE:
                        if(last_cmd == 255) set_gait_crawl(STRAIGHT);  
                        else set_gait_idle();
                        break;
                    case CPG_MODE_STANDBY:
                        set_gait_standby();
                        break;
                    case MODE_TURTLE:
                        set_gait_trot(STRAIGHT);
                        break;
                    case MODE_CRAWL:
                        set_gait_crawl(STRAIGHT);
                        break;
                    case MODE_CREEP:
                        set_gait_creep(STRAIGHT);
                        break;
                    case MODE_TROT_LEFT:
                        set_gait_trot(LEFT);
                        break;
                    case MODE_TROT_RIGHT:
                        set_gait_trot(RIGHT);
                        break;
                    case MODE_CREEP_LEFT:
                        set_gait_creep(LEFT);
                        break;
                    case MODE_CREEP_RIGHT:
                        set_gait_creep(RIGHT);
                        break;
                    case MODE_CRAWL_LEFT:
                        set_gait_crawl(LEFT);
                       break;
                    case MODE_CRAWL_RIGHT:
                        set_gait_crawl(RIGHT);
                       break;
                    case LEG_ORIENTATION_INVERTED:
                        set_system_idle();
                        break;
                    case LEG_ORIENTATION_NORMAL:
                        set_system_idle();
                        break;
                    default:
                        set_gait_idle();
                        break;
                }

                last_cmd = cmd;
                xSemaphoreGive(cpg_params_mutex);
            } else {
                ESP_LOGE(TAG_SEQ, "Sequence mutex timeout!");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Hardware timer setup
void hardware_Timer_setup(void) {
    esp_timer_create_args_t timer_args = {
        .callback = &cpg_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "cpg_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &cpg_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(cpg_timer_handle, CPG_UPDATE_RATE_US));
    ESP_LOGI(TAG_CPG, "CPG timer started at 2500Hz");
}

// CPG app entry
void CPG_app_main(void) {
    ESP_LOGI(TAG_CPG, "Starting CPG application...");
    cpg_params_mutex = xSemaphoreCreateMutex();
    if (!cpg_params_mutex) {
        ESP_LOGE(TAG_CPG, "Failed to create CPG mutex");
        return;
    }

    // Init phases
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
        cpg_network[i].phase = 0.0f;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    set_gait_idle(); // Safe start
    Update_Oscillator_base_parameters();
    // Create tasks
    xTaskCreatePinnedToCore(command_runner_task, "seq_runner", 4096, NULL, 18, &seq_task_handle, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreatePinnedToCore(cpg_task, "cpg_update", 4096, NULL, 15, &cpg_task_handle, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    hardware_Timer_setup();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG_CPG, "CPG application started");
}