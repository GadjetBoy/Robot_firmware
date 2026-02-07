#include "motor_command.h" 
#include "BLE.h"
#include "PID.h"

#define TAG "CPG"
// CPG update loop timing
#define CPG_UPDATE_RATE_MS 20 // 50 Hz update rate
#define CPG_DT (CPG_UPDATE_RATE_MS / 1000.0f) // 0.02f seconds

// Define the global CPG variables
Oscillator cpg_network[NUM_OSCILLATORS];
osc_pram pram;
volatile SequenceMode current_mode;
float coupling_weights[NUM_OSCILLATORS][NUM_OSCILLATORS];
float phase_offsets[NUM_OSCILLATORS][NUM_OSCILLATORS];


void cpg_update_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CPG_UPDATE_RATE_MS);

    float d_phi[NUM_OSCILLATORS]; // To store the change in phase for each oscillator

    while (1) {
        // --- 1. Calculate phase changes for all oscillators ---
        for (int i = 0; i < NUM_OSCILLATORS; i++) {
            float coupling_term = 0.0f;
            
            // Sum the influence of all *other* oscillators (j) on this one (i)
            for (int j = 0; j < NUM_OSCILLATORS; j++) {
                if (i == j) continue; // An oscillator doesn't couple to itself
                
                coupling_term += coupling_weights[i][j] * sinf(cpg_network[j].phase - cpg_network[i].phase - phase_offsets[i][j]);
            }
            
            // The final "change in phase" formula
            d_phi[i] = cpg_network[i].omega + coupling_term;
        }

        // --- 2. Update all phases and calculate outputs ---
        // (We do this in a separate loop to prevent using "new" values
        // in the calculation for the same time step)
        for (int i = 0; i < NUM_OSCILLATORS; i++) {
            
            // Apply the change: new_phase = old_phase + (change * time_step)
            cpg_network[i].phase += d_phi[i] * CPG_DT;

            // Wrap the phase (keep it between 0 and 2*PI)
            cpg_network[i].phase = fmodf(cpg_network[i].phase, 2.0f * M_PI);
            if (cpg_network[i].phase < 0.0f) {
                cpg_network[i].phase += 2.0f * M_PI;
            }

            // --- 3. Calculate the final motor setpoint ---
            cpg_network[i].output = cpg_network[i].offset + 
                                    cpg_network[i].amplitude * sinf(cpg_network[i].phase);

            ESP_LOGE(TAG, "Float[%d] = %.2f", i, cpg_network[i].output);
            
            // --- 4. Send the new setpoint to the PID task ---
            // This replaces your old run_crawl_gait logic.
            // Your PID task will now get a constant stream of new targets.
            if (cpg_network[i].amplitude != 0) { // Only send if not idle
                 motor_set(i, (int32_t)cpg_network[i].output, true);
            } else {
                 motor_set(i, (int32_t)cpg_network[i].offset, true); // Go to offset and deactivate
            }
        }
        
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// A helper function to set parameters for an oscillator
void set_oscillator_params(int i, float omega, float amp, float offset) {
    cpg_network[i].omega = omega;
    cpg_network[i].amplitude = amp;
    cpg_network[i].offset = offset;
}

// Call this from sequence_runner_task to set the IDLE gait
void set_gait_idle() {
    // Stop all oscillators by setting their amplitude to 0
    for(int i=0; i < NUM_OSCILLATORS; i++) {
        // Use a default offset (e.g., 0) or a "stand" position
        float stand_offset = 0;
        taskENTER_CRITICAL(&ble_mux); 
        if (i % 2 != 0) stand_offset = float_val[3];// Example: knees bent
        taskEXIT_CRITICAL(&ble_mux);

        set_oscillator_params(i, 0, 0, stand_offset); // Freq=0, Amp=0
    }
    // No coupling needed
    memset(coupling_weights, 0, sizeof(coupling_weights));
    memset(phase_offsets, 0, sizeof(phase_offsets));
}

/*void Set_Oscillator_base_parameters(void){
// --- 2. Set Oscillator base parameters ---
    taskENTER_CRITICAL(&ble_mux);
    float base_freq = 2.0f * M_PI * float_val[4]; // 1.0 Hz (1 cycle/sec)
    float hip_amp = float_val[0];  // Example: 1500 counts swing
    float knee_amp = float_val[1];   // Example: 800 counts lift
    float hip_offset = float_val[2];
    float knee_offset = float_val[3]; // Example: knee center position
    taskEXIT_CRITICAL(&ble_mux);
}*/

// Call this to set the TROT gait
void set_gait_trot() {
    // --- 1. Reset all couplings ---
    memset(coupling_weights, 0, sizeof(coupling_weights));
    memset(phase_offsets, 0, sizeof(phase_offsets));

    // --- 2. Set Oscillator base parameters ---
    float base_freq = 2.0f * M_PI * 1.0f; // 1.0 Hz (1 cycle/sec)
    float hip_amp = 32704.0f;  // Example: 1500 counts swing
    float knee_amp = 16352.0f;   // Example: 800 counts lift
    float hip_offset = 0.0f;
    float knee_offset = 1200.0f; // Example: knee center position

    // Set params for all Hips
    set_oscillator_params(FLH, base_freq, hip_amp, hip_offset);
    set_oscillator_params(FRH, base_freq, hip_amp, hip_offset);
    set_oscillator_params(BLH, base_freq, hip_amp, hip_offset);
    set_oscillator_params(BRH, base_freq, hip_amp, hip_offset);
    
    // Set params for all Knees
    set_oscillator_params(FLK, base_freq, -knee_amp, knee_offset);
    set_oscillator_params(FRK, base_freq, knee_amp, knee_offset);
    set_oscillator_params(BLK, base_freq, -knee_amp, knee_offset);
    set_oscillator_params(BRK, base_freq, knee_amp, knee_offset);

    // --- 3. Define Coupling ---
    float K_intra_leg = 10.0f; // Strong coupling between a hip and its knee
    float K_inter_leg = 5.0f;  // Medium coupling between legs

    // === INTRA-LEG Coupling (Hip to Knee) ===
    // We want the knee to lift (e.g., sin(phi)=1) *before* the hip
    // moves forward (e.g., sin(phi)=0 -> 1).
    // A -90 degree (-PI/2) offset on the knee relative to the hip is common.
    //if(KH_offset==0){
     float  KH_offset =2.0f;
    //}

    float knee_lag = -M_PI / KH_offset;// Knee lags hip by 90 deg
    // FL leg
    coupling_weights[FLK][FLH] = K_intra_leg; // Knee listens to Hip
    phase_offsets[FLK][FLH] = knee_lag;
    // FR leg
    coupling_weights[FRK][FRH] = K_intra_leg;
    phase_offsets[FRK][FRH] = knee_lag;
    // BL leg
    coupling_weights[BLK][BLH] = K_intra_leg;
    phase_offsets[BLK][BLH] = knee_lag;
    // BR leg
    coupling_weights[BRK][BRH] = K_intra_leg;
    phase_offsets[BRK][BRH] = knee_lag;
    
    // === INTER-LEG Coupling (Hip to Hip) ===
    // Trot: Diagonals are in-phase (0 offset)
    //       Opposing diagonals are out-of-phase (PI offset)

    // FLH (0) <-> BRH (6) :: In-phase (0)
    coupling_weights[FLH][BRH] = K_inter_leg;
    phase_offsets[FLH][BRH] = 0;
    coupling_weights[BRH][FLH] = K_inter_leg;
    phase_offsets[BRH][FLH] = 0;
    
    // FRH (2) <-> BLH (4) :: In-phase (0)
    coupling_weights[FRH][BLH] = K_inter_leg;
    phase_offsets[FRH][BLH] = 0;
    coupling_weights[BLH][FRH] = K_inter_leg;
    phase_offsets[BLH][FRH] = 0;
    
    // FLH (0) <-> FRH (2) :: Out-of-phase (PI)
    coupling_weights[FLH][FRH] = K_inter_leg;
    phase_offsets[FLH][FRH] = M_PI;
    coupling_weights[FRH][FLH] = K_inter_leg;
    phase_offsets[FRH][FLH] = M_PI;
    
    // ... (and so on for all other hip-to-hip pairs) ...
    // FLH(0) <-> BLH(4) :: Out-of-phase (PI)
    coupling_weights[FLH][BLH] = K_inter_leg;
    phase_offsets[FLH][BLH] = M_PI;
    coupling_weights[BLH][FLH] = K_inter_leg;
    phase_offsets[BLH][FLH] = M_PI;

    // FRH(2) <-> BRH(6) :: Out-of-phase (PI)
    coupling_weights[FRH][BRH] = K_inter_leg;
    phase_offsets[FRH][BRH] = M_PI;
    coupling_weights[BRH][FRH] = K_inter_leg;
    phase_offsets[BRH][FRH] = M_PI;

    // BLH(4) <-> BRH(6) :: Out-of-phase (PI)
    coupling_weights[BLH][BRH] = K_inter_leg;
    phase_offsets[BLH][BRH] = M_PI;
    coupling_weights[BRH][BLH] = K_inter_leg;
    phase_offsets[BRH][BLH] = M_PI;

    // ... etc. You must define all pairs for a stable network.
}

uint8_t get_mode(void){
  taskENTER_CRITICAL(&ble_mux);
  current_mode =(SequenceMode)byte_val[0];
  taskEXIT_CRITICAL(&ble_mux);
  return (uint8_t)current_mode;
}


void sequence_runner_task(void *arg) {
    uint8_t mode;
    uint8_t last_mode = 255; // Force an update on first run

    while (1) {
        // get_set_point(); // You might not need this anymore,
                         // Amplitudes/Offsets are set by the gait function.
                         // You could use BLE to tune them, though!

        mode = get_mode();

        if (mode != last_mode) { // Only update when the mode *changes*
            ESP_LOGI(TAG, "Mode change detected: %d", mode);
            switch (mode) {
                case MODE_CRAWL:
                    // You'll need to write set_gait_crawl()
                    // set_gait_crawl();
                    set_gait_trot(); 
                    ESP_LOGI(TAG, "Setting CRAWL gait");
                    break;

                case MODE_TURTLE: // Let's map this to TROT
                    set_gait_trot();
                    ESP_LOGI(TAG, "Setting TROT gait");
                    break;

                case MODE_STANDBY:
                    break;
                case MODE_IDLE:
                    set_gait_idle();
                    ESP_LOGI(TAG, "Setting IDLE gait");
                    break;

                default:
                    // Maybe just default to IDLE
                    set_gait_idle();
                    break;
            }
            last_mode = mode; // Remember the last mode
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Check for a new mode 10 times/sec
    }
}

void mot_command_app_main(void) {

    ESP_LOGI(TAG, "settingup the motor comnads");

    // --- NEW CPG Setup ---
    ESP_LOGI(TAG, "Initializing CPG network...");
    // Initialize to a safe state
    set_gait_idle(); 
    // ---

    TaskHandle_t seq_runner_handle = NULL;

   xTaskCreatePinnedToCore(sequence_runner_task, 
                            "seq_runner",
                             4096, 
                             NULL, 
                             7, 
                             &seq_runner_handle, 
                             1);

    // --- NEW CPG Task ---
    TaskHandle_t cpg_task_handle = NULL;
    xTaskCreatePinnedToCore(cpg_update_task, 
                            "cpg_updater", 
                            4096, // CPG task might need stack for float math
                            NULL, 
                            8, // Higher priority than sequence runner
                            &cpg_task_handle, 
                            1); // Pin to core 1
    // ---
    
    ESP_LOGI(TAG, "settingup the motor comnads complete");
}