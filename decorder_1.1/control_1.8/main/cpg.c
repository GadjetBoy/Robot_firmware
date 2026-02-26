#include "cpg.h"
#include "uart.h"
#include "pcnt.h"
#include "ble.h"
#include "pid.h"
#include "gate.h"

//============================================================PID==================================================================
#define PID_QUEUE_LENGTH 1

// Math Constants
#define TWO_PI (2.0f * 3.1415926535f)
#define CREEP_OFFST_MODI (1000.0f)

// Tags for logging (consistent)
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

float SMOOTH_ALPHA = 0.050f;

// Per-motor homing invert: -1 = flip output sign (fix wrong-direction joints), 1 = normal
//static const int8_t HOMING_INVERT[NUM_MOTORS] = { 1, 1, 1, 1, 1, 1, 1, 1 };  

//volatile SequenceMode pending_mode = MODE_IDLE;
//volatile bool mode_change_pending = false;

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
        /* IDLE: stop all motors (output 0) */
        if (cpg_run_mode == CPG_MODE_IDLE) {
            pid_outputs[i] = 0.0f;
        } 
        else if (cpg_run_mode == CPG_MODE_HOMING) {
            /* HOMING: drive to target 0 with min duty to overcome stiction */
            PIDController_Update(&motors[i].pos_pid, motors[i].target_position,
                                motors[i].current_position, (uint8_t)i);

            pid_outputs[i] = motors[i].pos_pid.output;

            float abs_pos = fabsf((float)motors[i].current_position);

            if (abs_pos > STOP_THRESHOLD*2) {
                float abs_out = fabsf(pid_outputs[i]);
                float min_duty = (abs_pos > 100) ? (MIN_DUTY * 1.5f) : MIN_DUTY;

                if (abs_out > 0.01f && abs_out < min_duty) {
                    float sign = (pid_outputs[i] >= 0.0f) ? 1.0f : -1.0f;
                    pid_outputs[i] = sign * min_duty;
                }
            }

            if (abs_pos <= STOP_THRESHOLD*2) pid_outputs[i] = 0.0f;
        } 
        else if (cpg_run_mode == CPG_MODE_STANDBY) {
            /* STANDBY: hold at target (hip 0, knee -18000) with min duty to overcome stiction */
            PIDController_Update(&motors[i].pos_pid, motors[i].target_position,
                                 motors[i].current_position, (uint8_t)i);
            pid_outputs[i] = motors[i].pos_pid.output;
            float err = motors[i].target_position - (float)motors[i].current_position;
            float abs_err = fabsf(err);
            if (abs_err > TOL) {
                float abs_out = fabsf(pid_outputs[i]);
                float min_duty = (abs_err > 200) ? (MIN_DUTY * 2.0f) : MIN_DUTY;
                if (abs_out > 0.01f && abs_out < min_duty) {
                    float sign = (pid_outputs[i] >= 0.0f) ? 1.0f : -1.0f;
                    pid_outputs[i] = sign * min_duty;
                }
            }
            if (abs_err <= TOL) pid_outputs[i] = 0.0f;
        }
        else {
            PIDController_Update(&motors[i].pos_pid, motors[i].target_position,
                                 motors[i].current_position, (uint8_t)i);
            pid_outputs[i] = motors[i].pos_pid.output;
            float abs_pos = fabsf((float)motors[i].current_position);
            if (!motors[i].active && abs_pos <= STOP_THRESHOLD*2) pid_outputs[i] = 0.0f;
        }

       
        motors[i].last_enc_val = motors[i].current_position;
    }
    // Send PID outputs to UART via double buffer
    update_uart_motor_commands(pid_outputs);

    xTaskNotifyGive(uart_send_task);
       
    xQueueOverwrite(encorderQue,filtered_enc);
    
}

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
    if (cmd == LEG_ORIENTATION_NORMAL)
        leg_orientation = LEG_ORIENTATION_NORMAL;
    else if (cmd == LEG_ORIENTATION_INVERTED)
        leg_orientation = LEG_ORIENTATION_INVERTED;
}

static inline uint8_t get_commands(void) {

    taskENTER_CRITICAL(&ble_mux);
    cmd = byte_val[0];
    taskEXIT_CRITICAL(&ble_mux);

    update_orientation_from_ble(cmd);

    if (cmd == MODE_PIVOT_TURN || cmd == MODE_PIVOT_CRAWL) {
        pivot_turn = true;
    } else {
        pivot_turn = false;
    }

    return cmd;
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

    // Sign flip for right legs (FRH,BRH) during pivot turn - left/right move opposite
    bool is_right_leg = (i == FRH || i == BRH);  // Adjust indices if mapping differs
    if (is_right_leg && pivot_turn == false) {
        signed_target = -signed_target;
    }
    
    /* Creep: Prevent backward pull only on non-diagonal stance knees. Let diagonal stance knee bend more (tilt toward stride triangle). */
    bool is_knee = (i == FLK || i == FRK || i == BLK || i == BRK);
    bool is_creep = (last_cmd == MODE_CREEP || last_cmd == MODE_CREEP_LEFT || last_cmd == MODE_CREEP_RIGHT);
    if (is_knee && is_creep) {
        /* Find swinging leg (hip phase < π) and diagonal stance knee (allowed to bend more) */
        int swing_hip = -1;  /* which hip is swinging */
        if (cpg_network[FLH].phase < M_PI) swing_hip = FLH;
        else if (cpg_network[FRH].phase < M_PI) swing_hip = FRH;
        else if (cpg_network[BLH].phase < M_PI) swing_hip = BLH;
        else if (cpg_network[BRH].phase < M_PI) swing_hip = BRH;
        int diag_knee = (swing_hip == FLH) ? BRK : (swing_hip == FRH) ? BLK : (swing_hip == BLH) ? FRK : (swing_hip == BRH) ? FLK : -1;
        int same_leg_hip = (i == FLK) ? FLH : (i == FRK) ? FRH : (i == BLK) ? BLH : BRH;
        bool is_swing_knee = (same_leg_hip == swing_hip);
        bool is_diag_stance_knee = (i == diag_knee);
        /* Only clamp the two non-diagonal stance knees; allow swing + diagonal stance to bend freely */
        if (!is_swing_knee && !is_diag_stance_knee) {
            float knee_offset = cpg_network[i].offset;
            if (signed_target < knee_offset) signed_target = knee_offset;
        }
        else {
            float knee_offset = (cpg_network[i].offset - CREEP_OFFST_MODI);
            if (signed_target < knee_offset) signed_target = knee_offset;
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
    static bool idle_reset_done = false;

    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        start = esp_timer_get_time();
        if (xSemaphoreTake(cpg_params_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGE(TAG_CPG, "CPG mutex timeout!");
            continue;
        }

        if (cpg_run_mode == CPG_MODE_IDLE) {
            vTaskDelay(pdMS_TO_TICKS(2));
            for (int i = 0; i < NUM_MOTORS; i++) motor_set((uint8_t)i, 0, false);
            xSemaphoreGive(cpg_params_mutex);
            run_position_loop();  /* outputs 0, motors stop */
            continue;
        }

        if (cpg_run_mode == CPG_MODE_HOMING) {
            if (!idle_reset_done) {
                for (int i = 0; i < NUM_MOTORS; i++) PIDController_Init(&motors[i].pos_pid);
                idle_reset_done = true;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
            for (int i = 0; i < NUM_MOTORS; i++) motor_set((uint8_t)i, 0, true);  /* active=true: drive to 0 */
            xSemaphoreGive(cpg_params_mutex);
            run_position_loop();
            continue;
        }

        if (cpg_run_mode == CPG_MODE_STANDBY) {
            if (!idle_reset_done) {
                for (int i = 0; i < NUM_MOTORS; i++) {
                    PIDController_Init(&motors[i].pos_pid);
                }
                idle_reset_done = true;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
            CPG_network_pram.hip_offset  = 0.0f;
            CPG_network_pram.knee_offset = -18000.0f;

            for (int i = 0; i < NUM_OSCILLATORS; i++) {
              float offset = (i % 2 == 0) ? CPG_network_pram.hip_offset: CPG_network_pram.knee_offset;
              motor_set((uint8_t)i, offset, true);
            }

            xSemaphoreGive(cpg_params_mutex);
            run_position_loop();
            continue;
        }

        idle_reset_done = false;

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

        smooth_gait_parameters(SMOOTH_ALPHA);

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

          float warped_omega = cpg_network[i].omega * alpha;

          float coupling_term = 0.0f;
          for (int j = 0; j < NUM_OSCILLATORS; j++) {
              if (i == j) continue;
              float sin_term = sinf(phase[j] - phase[i] - phase_offsets[i][j]);
              float norm_amp = cpg_network[i].amplitude / CPG_network_pram.max_amp;

               coupling_term += coupling_weights[i][j] * norm_amp * sin_term;
            }

          // Apply the alpha multiplier to the base frequency only
          float damping = CPG_network_pram.damping;
          d_phi[i] = (1.0f - damping) * warped_omega + damping * coupling_term;
        }

        // 2. Update phases and outputs
        for (int i = 0; i < NUM_OSCILLATORS; i++) {

            cpg_network[i].phase += d_phi[i] * CPG_DT;

            cpg_network[i].phase = fmodf(cpg_network[i].phase, TWO_PI);
            if (cpg_network[i].phase < 0.0f) cpg_network[i].phase += TWO_PI;

            float target_amp = (i % 2 == 0) ?
                ((i == FLH || i == BLH) ? CPG_network_pram.hip_amp_left : CPG_network_pram.hip_amp_right) :
                ((i == FLK || i == BLK) ? CPG_network_pram.knee_amp_left : CPG_network_pram.knee_amp_right);
            if (fabsf(target_amp - cpg_network[i].amplitude) > 1.0f) {
                cpg_network[i].amplitude += 0.008f * (target_amp - cpg_network[i].amplitude) * CPG_DT;
            }

            // --- Bias correction (prevent drift) ---
            float target_offset = (i % 2 != 0) ? CPG_network_pram.knee_offset : CPG_network_pram.hip_offset;
            cpg_network[i].offset += 0.050f * (target_offset - cpg_network[i].offset) * CPG_DT;

            /* Creep: boost diagonal stance knee when opposite leg swings → tilt toward stride triangle for stability */
            float eff_amp = cpg_network[i].amplitude;
            /*if (CPG_network_pram.diagonal_knee_boost > 1.0f && (i == FLK || i == FRK || i == BLK || i == BRK)) {
                 Diagonal pairs: FL↔BR, FR↔BL. Knee K boosts when diagonal hip H is in swing [0,π] 
                int diag_hip = (i == FLK) ? BRH : (i == FRK) ? BLH : (i == BLK) ? FRH : (i == BRK) ? FLH : -1;
                if (diag_hip >= 0 && cpg_network[diag_hip].phase < M_PI) {
                    eff_amp *= CPG_network_pram.diagonal_knee_boost;
                }
            }*/
            cpg_network[i].output = cpg_network[i].offset + eff_amp * sinf(cpg_network[i].phase);

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
        /* For posture commands: only act when posture actually changed (avoids re-running on repeated BLE sends) */
        bool is_posture_cmd = (cmd == BODY_POSTURE_NORMAL || cmd == BODY_POSTURE_LOW || cmd == BODY_POSTURE_CROUCH);
        bool posture_changed = is_posture_cmd && ((body_posture_t)cmd != posture);
        bool mode_changed = (cmd != last_cmd);
        bool should_act = is_posture_cmd ? posture_changed : mode_changed;
        if (should_act) {
            if (!is_posture_cmd) ESP_LOGE(TAG_SEQ, "Mode change: %d -> %d", last_cmd, cmd);
            // Take mutex, set gait, release mutex QUICKLY
            if (xSemaphoreTake(cpg_params_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                switch (cmd) {
                    case CPG_MODE_HOMING:
                        set_gait_homing();
                        break;
                    case CPG_MODE_STANDBY:
                        set_gait_standby();
                        break;
                    case CPG_MODE_IDLE:
                        set_gait_idle();
                        break;
                    case MODE_TURTLE:
                        set_gait_trot(STRAIGHT, posture);
                        break;
                    case MODE_CRAWL:
                        set_gait_crawl(STRAIGHT, posture);
                        break;
                    case MODE_CREEP:
                        set_gait_creep(STRAIGHT, posture);
                        break;
                    case MODE_TROT_LEFT:
                        set_gait_trot(LEFT, posture);
                        break;
                    case MODE_TROT_RIGHT:
                        set_gait_trot(RIGHT, posture);
                        break;
                    case MODE_CREEP_LEFT:
                        set_gait_creep(LEFT, posture);
                        break;
                    case MODE_CREEP_RIGHT:
                        set_gait_creep(RIGHT, posture);
                        break;
                    case MODE_CRAWL_LEFT:
                        set_gait_crawl(LEFT, posture);
                        break;
                    case MODE_CRAWL_RIGHT:
                        set_gait_crawl(RIGHT, posture);
                        break;
                    case MODE_PIVOT_TURN:
                        set_gait_trot(STRAIGHT, posture);
                        break;
                    case MODE_PIVOT_CRAWL:
                        set_gait_crawl(STRAIGHT, posture);
                        break;
                    case LEG_ORIENTATION_INVERTED:
                        set_system_idle();
                        break;
                    case LEG_ORIENTATION_NORMAL:
                        set_system_idle();
                        break;
                    case BODY_POSTURE_NORMAL:
                    case BODY_POSTURE_LOW:
                    case BODY_POSTURE_CROUCH: {
                        posture = (body_posture_t)cmd;
                        switch (last_cmd) {
                            case MODE_TURTLE:
                            case MODE_PIVOT_TURN:
                                set_gait_trot(STRAIGHT, posture);
                                break;
                            case MODE_TROT_LEFT:
                                set_gait_trot(LEFT, posture);
                                break;
                            case MODE_TROT_RIGHT:
                                set_gait_trot(RIGHT, posture);
                                break;
                            case MODE_CRAWL:
                            case MODE_PIVOT_CRAWL:
                                set_gait_crawl(STRAIGHT, posture);
                                break;
                            case MODE_CRAWL_LEFT:
                                set_gait_crawl(LEFT, posture);
                                break;
                            case MODE_CRAWL_RIGHT:
                                set_gait_crawl(RIGHT, posture);
                                break;
                            case MODE_CREEP:
                                set_gait_creep(STRAIGHT, posture);
                                break;
                            case MODE_CREEP_LEFT:
                                set_gait_creep(LEFT, posture);
                                break;
                            case MODE_CREEP_RIGHT:
                                set_gait_creep(RIGHT, posture);
                                break;
                            default:
                                set_gait_crawl(STRAIGHT, posture);
                                break;
                        }
                        break;
                    }
                    default:
                        set_gait_idle();
                        break;
                }

                //smooth_gait_parameters(SMOOTH_ALPHA);

                if (cmd != BODY_POSTURE_NORMAL && cmd != BODY_POSTURE_LOW && cmd != BODY_POSTURE_CROUCH) {
                    last_cmd = cmd;
                }
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

    pid_app_main();
    vTaskDelay(pdMS_TO_TICKS(10));
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