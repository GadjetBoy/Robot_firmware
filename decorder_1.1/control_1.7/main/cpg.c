#include "pcnt.h"
#include "cpg.h"
#include "UART.h"
#include "ble.h"

static float CPG_frequency = 0.090f;

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
//Motor motor_log[NUM_MOTORS];
//QueueHandle_t motor_command_queue;
//TaskHandle_t pid_loop_task = NULL;

volatile DRAM_ATTR float motor_cmd_buffer[2][NUM_MOTORS];
volatile DRAM_ATTR uint8_t current_write_index = 0;
portMUX_TYPE pid_buffer_swap_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t stall_count[NUM_MOTORS] = {0};

// CPG globals
Oscillator cpg_network[NUM_OSCILLATORS];
osc_pram CPG_network_pram;
volatile SequenceMode current_mode = MODE_IDLE;
float coupling_weights[NUM_OSCILLATORS][NUM_OSCILLATORS];
float phase_offsets[NUM_OSCILLATORS][NUM_OSCILLATORS];
static SemaphoreHandle_t cpg_params_mutex;

static esp_timer_handle_t cpg_timer_handle;
static TaskHandle_t cpg_task_handle = NULL;
static TaskHandle_t seq_task_handle = NULL;
static uint64_t start;

// ====================== Utility Functions ======================
void update_uart_motor_commands(float *new_commands) {
    uint8_t write_buffer = 1 - current_write_index; // Write to opposite buffer
   
    // Copy to back buffer
    memcpy((void*)motor_cmd_buffer[write_buffer], new_commands, sizeof(float) * NUM_MOTORS);
    // Atomic swap - ensures PID sees consistent snapshot
    taskENTER_CRITICAL(&pid_buffer_swap_mux);
    current_write_index = write_buffer;
    taskEXIT_CRITICAL(&pid_buffer_swap_mux);
}

static uint8_t get_mode(void) {
    taskENTER_CRITICAL(&ble_mux);
    current_mode = (SequenceMode)byte_val[0];
    taskEXIT_CRITICAL(&ble_mux);
    return (uint8_t)current_mode;
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
    if (xSemaphoreTake(uart_mutex,0) == pdTRUE) {
        float freq_scale = fminf(CPG_frequency / 0.1f, 2.5f);  // FIXED: Cap at 2.5x (less aggressive than 3.0)
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].pos_pid.Kp = update_motors[i].pos_pid.Kp * fminf(1.0f + 0.3f * freq_scale, 2.5f);  // Cap eff Kp<=5
            motors[i].pos_pid.Kd = update_motors[i].pos_pid.Kd * (1.0f + 0.6f * freq_scale);  // Milder Kd scale
            motors[i].pos_pid.Ki = fmaxf(update_motors[i].pos_pid.Ki * 0.8f, 0.005f);  // FIXED: Floor at 0.005, less reduction
            motors[i].pos_pid.limMaxInt = 150.0f / freq_scale;  // FIXED: Tighter (200→150) to limit windup
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
    if(fabs(pid->error)<=STOP_THRESHOLD*10){
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
    
    /*static uint32_t sat_count = 0;
    if (fabsf(pid->output) >= PWM_MAX * 0.95f) sat_count++;
    if (sat_count % 10000 == 0) ESP_LOGW("PID_SAT", "Motor[%d] saturated %lu%% of cycles", i, (sat_count * 100 / 1000));*/
}

// ====================== Position Loop ======================
void run_position_loop(){
    static int current_encoders[NUM_MOTORS] = {0}; // Local snapshot
    static int filtered_enc[NUM_MOTORS] = {0}; // IIR filter state
    static uint32_t log_counter = 0;
    log_counter++;
    //start = esp_timer_get_time();
    // Update gains periodically (throttled)
    //if (log_counter % 250 == 0) { // ~0.1s at 2500Hz
        update_PID_gain();
   // }
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

        if (!motors[i].active) {
            if (fabsf((float)motors[i].current_position) <= STOP_THRESHOLD * 5) {
                pid_outputs[i] = 0.0f;
            }
            continue;
        }

        // Stall detection
        float abs_err = fabsf(motors[i].pos_pid.error);
        if (abs_err > TOL) {
            if (motors[i].current_position == motors[i].last_enc_val) {
                stall_count[i]++;
                if (stall_count[i] >= 5) {
                    //ESP_LOGW(TAG_PID, "Stall detected on motor %d", i);
                    stall_count[i] = 5; // Cap
                    //pid_outputs[i] = PWM_MAX;
                }
            } else {
                stall_count[i] = 0;
            }
        }
        motors[i].last_enc_val = motors[i].current_position;
    }
    // Send PID outputs to UART via double buffer
    update_uart_motor_commands(pid_outputs);

    xTaskNotifyGive(uart_send_task);
        //static uint32_t pid_log_counter = 0;
        //pid_log_counter++;
       /* if (pid_log_counter % 100 == 0) { // ~50ms
             ESP_LOGI("PID", "Motor[0] error=%.0f, output=%.1f; Motor[4] error=%.0f, output=%.1f",
             motors[0].pos_pid.error, motors[0].pos_pid.output,
             motors[4].pos_pid.error, motors[4].pos_pid.output);
        }*/
       
    xQueueOverwrite(encorderQue,current_encoders);
        // PID work
        uint64_t total_elapsed = esp_timer_get_time() - start;
       // if (elapsed > CPG_UPDATE_RATE_US) ESP_LOGW(TAG_PID, "CPG and PID loop took %llu µs", elapsed);
        /*if (log_counter % 500 == 0) {
        ESP_LOGI("LOOP", "Elapsed μs=%.1f, Freq=%.2f, Hip amp=%.0f, Knee amp=%.0f", total_elapsed, CPG_frequency, 
                 CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
        }*/
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
        motors[i].pos_pid.limMinInt = -400.0f;
        motors[i].pos_pid.limMaxInt = 400.0f;
        PIDController_Init(&motors[i].pos_pid);
    }
    //xTaskCreatePinnedToCore(position_loop_task, "pos_loop", 5120, NULL, 20, &pid_loop_task,1);
    ESP_LOGI(TAG_PID, "PID application started successfully");
}

// ====================== CPG Functions ======================
// Set motor target with feedforward velocity term
void motor_set(uint8_t i, float target, bool set) {
    if (i >= NUM_MOTORS) return;

    //Sign flip for right legs (FR/BR: indices 1,3,5,7) for symmetry (forward gait)
    bool is_right_leg = (i == FRK || i == BRK);  // Adjust indices if mapping differs
    float signed_target = is_right_leg ? -target : target;  // Mirror right for opposite swing

    // Velocity FF (on signed target for correct direction)
    float phase = cpg_network[i].phase;
    float vel_ff = cpg_network[i].amplitude * cpg_network[i].omega * cosf(phase);
    motors[i].target_position = signed_target + (vel_ff * PID_DT * 0.6f);  // Keep your tuned gain
    motors[i].active = set;
    motors[i].prev_target = signed_target;

    if (ENABLE_MOTOR_DEBUG) {
        ESP_LOGI(TAG_MOTOR, "Motor %d target=%.2f (active=%s)", i, target, set ? "true" : "false");
    }
}

// Timer ISR (notifies task)
void IRAM_ATTR cpg_timer_callback(void* arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(cpg_task_handle, &woken);
    if (woken) portYIELD_FROM_ISR();
}

// CPG update task (runs at 2500Hz)
void cpg_update_task(void *arg) {
    float d_phi[NUM_OSCILLATORS];
    float phase[NUM_OSCILLATORS];
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        start = esp_timer_get_time();
        if (xSemaphoreTake(cpg_params_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGE(TAG_CPG, "CPG mutex timeout!");
            continue;
        }
        // 1. Snapshot phases and compute coupling
        for (int i = 0; i < NUM_OSCILLATORS; i++) {
            phase[i] = cpg_network[i].phase;

            float coupling_term = 0.0f;
            for (int j = 0; j < NUM_OSCILLATORS; j++) {
                if (i == j) continue;

                float sin_term = sinf(phase[j] - phase[i] - phase_offsets[i][j]);
                float norm_amp = cpg_network[i].amplitude / CPG_network_pram.max_amp;

                coupling_term += coupling_weights[i][j] * norm_amp * sin_term;
            }
            // Blended omega
            float damping = 0.1f;
            d_phi[i] = (1.0f - damping) * cpg_network[i].omega + damping * coupling_term;
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
            cpg_network[i].offset += 0.0250f * (target_offset - cpg_network[i].offset) * CPG_DT;

            cpg_network[i].output = cpg_network[i].offset +
                                    cpg_network[i].amplitude * sinf(cpg_network[i].phase);

            motor_set((uint8_t)i, cpg_network[i].output,
                      cpg_network[i].amplitude != 0.0f);
        }
        xSemaphoreGive(cpg_params_mutex);
        // Trigger PID loop
        run_position_loop();

        uint64_t elapsed = esp_timer_get_time() - start;
        if (elapsed > CPG_UPDATE_RATE_US && ENABLE_CPG_DEBUG) {
            ESP_LOGW(TAG_CPG, "CPG update overrun: %llu us", elapsed);
        }
    }
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

// Gait: IDLE (all off)
void set_gait_idle(void) {
    
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
        float stand_offset = (i % 2 == 0) ? CPG_network_pram.hip_offset : CPG_network_pram.knee_offset;
        cpg_network[i].phase = 0.0f;
        set_oscillator_params(i, 0.0f, 0.0f, stand_offset);
    }
    memset(coupling_weights, 0, sizeof(coupling_weights));
    memset(phase_offsets, 0, sizeof(phase_offsets));

    ESP_LOGI(TAG_CPG, "Gait set to IDLE");
}

// Gait: TROT (diagonal sync, alias for MODE_TURTLE)
void set_gait_trot(void) {
    ESP_LOGI(TAG_CPG, "set_gait_trot START (mutex already held)");

    memset(coupling_weights, 0, sizeof(coupling_weights));
    memset(phase_offsets, 0, sizeof(phase_offsets));

    CPG_network_pram.base_freq = TWO_PI * CPG_frequency;
    CPG_network_pram.hip_amp = 16352.0f;
    CPG_network_pram.knee_amp = 32704.0f;
    CPG_network_pram.hip_offset = 0.0f;
    CPG_network_pram.knee_offset = 0.0f;
    CPG_network_pram.KH_offset = 4.0f;

    // FIXED: Milder scaling (10-15% max cut at 0.2 Hz; hips slightly more to ease inertia)
    float hip_freq_factor = fminf(1.0f / sqrtf(1.0f + 2.0f * CPG_frequency), 0.90f);  // Coeff 2.0f: ~12% cut
    float knee_freq_factor = fminf(1.0f / sqrtf(1.0f + 1.5f * CPG_frequency), 0.92f); // Coeff 1.5f: ~8% cut
    CPG_network_pram.hip_amp *= hip_freq_factor;
    CPG_network_pram.knee_amp *= knee_freq_factor;

    // FIXED: Lower mult for stability (1.5x knees = ~1.5 cycles; tune 1.2-1.8 to avoid pull)
    CPG_network_pram.hip_omega_mult = 1.2f;
    CPG_network_pram.knee_omega_mult = 1.8f;  // Subtle fast knees without distortion

   // Recompute max_amp
   CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
   if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Set oscillators
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);

    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);

    // Couplings
    float K_intra_leg = 8.0f;
    float K_inter_leg = 5.0f;
    float knee_lag = -TWO_PI / CPG_network_pram.KH_offset;

    // Intra-leg
    coupling_weights[FLK][FLH] = K_intra_leg; phase_offsets[FLK][FLH] = knee_lag;
    coupling_weights[FRK][FRH] = K_intra_leg; phase_offsets[FRK][FRH] = knee_lag;
    coupling_weights[BLK][BLH] = K_intra_leg; phase_offsets[BLK][BLH] = knee_lag;
    coupling_weights[BRK][BRH] = K_intra_leg; phase_offsets[BRK][BRH] = knee_lag;
    // Inter-leg diagonals in-phase
    coupling_weights[FLH][BRH] = K_inter_leg; phase_offsets[FLH][BRH] = 0;
    coupling_weights[BRH][FLH] = K_inter_leg; phase_offsets[BRH][FLH] = 0;
    coupling_weights[FRH][BLH] = K_inter_leg; phase_offsets[FRH][BLH] = 0;
    coupling_weights[BLH][FRH] = K_inter_leg; phase_offsets[BLH][FRH] = 0;
    // Out-of-phase pairs
    coupling_weights[FLH][FRH] = K_inter_leg; phase_offsets[FLH][FRH] = TWO_PI / 2.0f;
    coupling_weights[FRH][FLH] = K_inter_leg; phase_offsets[FRH][FLH] = TWO_PI / 2.0f;
    coupling_weights[FLH][BLH] = K_inter_leg; phase_offsets[FLH][BLH] = TWO_PI / 2.0f;
    coupling_weights[BLH][FLH] = K_inter_leg; phase_offsets[BLH][FLH] = TWO_PI / 2.0f;
    coupling_weights[FRH][BRH] = K_inter_leg; phase_offsets[FRH][BRH] = TWO_PI / 2.0f;
    coupling_weights[BRH][FRH] = K_inter_leg; phase_offsets[BRH][FRH] = TWO_PI / 2.0f;
    coupling_weights[BLH][BRH] = K_inter_leg; phase_offsets[BLH][BRH] = TWO_PI / 2.0f;
    coupling_weights[BRH][BLH] = K_inter_leg; phase_offsets[BRH][BLH] = TWO_PI / 2.0f;
    // Seed phases
    cpg_network[FLH].phase = 0.0f; cpg_network[BRH].phase = 0.0f;
    cpg_network[FLK].phase = TWO_PI / 4.0f; cpg_network[BRK].phase = TWO_PI / 4.0f;
    cpg_network[FRH].phase = TWO_PI / 2.0f; cpg_network[BLH].phase = TWO_PI / 2.0f;
    cpg_network[FRK].phase = TWO_PI / 2.0f + TWO_PI / 4.0f; cpg_network[BLK].phase = TWO_PI / 2.0f + TWO_PI / 4.0f;
    
    ESP_LOGI(TAG_CPG, "Gait set to TROT at freq=%.2f Hz", CPG_frequency);
}

// Gait: CRAWL (placeholder)
void set_gait_crawl(void) {
    set_gait_trot(); // Extend as needed
    ESP_LOGI(TAG_CPG, "Gait set to CRAWL");
}

// Gait: STANDBY (hold)
void set_gait_standby(void) {
    set_gait_idle();
    ESP_LOGI(TAG_CPG, "Gait set to STANDBY");
}

// Sequence runner (checks mode changes)
void sequence_runner_task(void *arg) {
    uint8_t mode;
    uint8_t last_mode = 255;
    while (1) {
        mode = get_mode();
        if (mode != last_mode) {
            ESP_LOGE(TAG_SEQ, "Mode change: %d -> %d", last_mode, mode);
            if (xSemaphoreTake(cpg_params_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                switch (mode) {
                    case MODE_IDLE:
                        set_gait_idle();
                        break;
                    case MODE_CRAWL:
                        set_gait_crawl();
                        break;
                    case MODE_STANDBY:
                        set_gait_standby();
                        break;
                    case MODE_TURTLE:
                        set_gait_trot();
                        break;
                    default:
                        set_gait_idle();
                        break;
                }
                last_mode = mode;
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
    xTaskCreatePinnedToCore(sequence_runner_task, "seq_runner", 4096, NULL, 9, &seq_task_handle, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreatePinnedToCore(cpg_update_task, "cpg_update", 4096, NULL, 20, &cpg_task_handle, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    hardware_Timer_setup();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG_CPG, "CPG application started");
}