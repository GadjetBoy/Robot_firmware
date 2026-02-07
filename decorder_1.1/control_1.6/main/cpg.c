#include "cpg.h" 
#include "ble.h"

#define TAG "CPG"
// CPG update loop timing
#define CPG_UPDATE_RATE_HZ 1000 //HZ
#define CPG_DT (1.0f / CPG_UPDATE_RATE_HZ)  //seconds
#define CPG_UPDATE_RATE_US CPG_DT*1000000.0f // microseconds
//#define 
const float TWO_PI = 2.0f * 3.1415926535f; // Use a float constant

// Define the global CPG variables
Oscillator cpg_network[NUM_OSCILLATORS];
osc_pram CPG_network_pram;
volatile SequenceMode current_mode;
float coupling_weights[NUM_OSCILLATORS][NUM_OSCILLATORS];
float phase_offsets[NUM_OSCILLATORS][NUM_OSCILLATORS];
static SemaphoreHandle_t cpg_params_mutex;

// Timer handle for high-frequency CPG updates
static esp_timer_handle_t cpg_pid_timer_handle;
static TaskHandle_t cpg_task_handle = NULL;
static QueueHandle_t cpg_update_queue = NULL;
static uint32_t counter_cpg_hz = 0;

// Power of 2 for efficient wrapping
static DRAM_ATTR motor_Command_t cmd[NUM_MOTORS];
DRAM_ATTR motor_Command_t cmd_buffer[CMD_BUFFER_SIZE][NUM_MOTORS];
volatile DRAM_ATTR uint8_t write_index = 0;
//volatile uint8_t read_index = 0;
portMUX_TYPE cpg_ring_mux = portMUX_INITIALIZER_UNLOCKED;

// CPG update command structure
typedef struct {
    uint32_t sequence;
} cpg_update_cmd_t;


void motor_set(uint8_t i, float target, bool set) {
    cmd[i].motor_target=target;
    cmd[i].activate= set;
}

void write_motor_commands(motor_Command_t *new_commands) {
    uint8_t write_buffer = 1 - write_index;  // Write to opposite buffer
    
    // Copy to back buffer
    memcpy((void*)cmd_buffer[write_buffer], new_commands, 
           sizeof(motor_Command_t) * NUM_MOTORS);
    
    // Atomic swap - ensures PID sees consistent snapshot
    taskENTER_CRITICAL(&cpg_ring_mux);
    write_index = write_buffer;
    taskEXIT_CRITICAL(&cpg_ring_mux);
}

// Timer callback function (runs in ISR context)
void IRAM_ATTR cpg_timer_callback(void* arg) {
    static uint32_t sequence_counter = 0;  // Static persists between calls
    

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send notification to i2c task
    vTaskNotifyGiveFromISR(pid_loop_task, &xHigherPriorityTaskWoken);

    cpg_update_cmd_t Cmd = {
        .sequence = sequence_counter
    };
    sequence_counter++;  

    counter_cpg_hz++;

    if (counter_cpg_hz >= 5) { // 2 * 100µs = 5 kHz
     // Send notification to CPG task
     vTaskNotifyGiveFromISR(cpg_task_handle, &xHigherPriorityTaskWoken);
    
     counter_cpg_hz = 0;
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void cpg_update_task(void *arg) {
    cpg_update_cmd_t update_cmd;
    uint32_t last_sequence = 0;
    float d_phi[NUM_OSCILLATORS]; // To store the change in phase for each oscillator

    while (1) {

       ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      if (xSemaphoreTake(cpg_params_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

        float phase[NUM_OSCILLATORS];

        for(int i =0;i<NUM_OSCILLATORS; i++){
           phase[i] = cpg_network[i].phase;  
        }
        // --- 1. Calculate phase changes for all oscillators ---
        for (int i = 0; i < NUM_OSCILLATORS; i++) { 

            float coupling_term = 0.0f;

            // --- Amplitude stabilization ---
            float target_amp = (i % 2 == 0) ? CPG_network_pram.hip_amp:CPG_network_pram.knee_amp;
            cpg_network[i].amplitude += 0.001f * (target_amp - cpg_network[i].amplitude)*CPG_DT; // smooth correction

            
            // --- Bias correction (prevent drift) ---
            float target_offset = (i % 2 != 0) ? CPG_network_pram.knee_offset:CPG_network_pram.hip_offset;
            cpg_network[i].offset += 0.005f * (target_offset - cpg_network[i].offset)*CPG_DT;
            
           
            // Sum the influence of all *other* oscillators (j) on this one (i)
            for (int j = 0; j < NUM_OSCILLATORS; j++) {
                if (i == j) continue; // An oscillator doesn't couple to itself
                
                coupling_term += (coupling_weights[i][j])*(sinf(phase[j] - phase[i] - phase_offsets[i][j]));
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
            cpg_network[i].phase = fmodf(cpg_network[i].phase, TWO_PI);
            if (cpg_network[i].phase < 0.0f) cpg_network[i].phase += TWO_PI;

            // --- 3. Calculate the final motor setpoint ---
            cpg_network[i].output = cpg_network[i].offset + 
                                    cpg_network[i].amplitude * sinf(cpg_network[i].phase);

            if (cpg_network[i].amplitude != 0) {
                motor_set(i, cpg_network[i].output, true);
            } else {
                motor_set(i, cpg_network[i].offset, false);
            }
        
        } 
        // --- Give the Mutex Back ---
            xSemaphoreGive(cpg_params_mutex);

        // Only send the command array if we successfully updated it.
          write_motor_commands(cmd);

        }else {
            ESP_LOGE(TAG, "CPG task failed to get mutex!");
        }

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
        if (i % 2 != 0) stand_offset = 0;// Example: knees bent

        cpg_network[i].phase = 0;        
        cpg_network[i].output = stand_offset; 

        set_oscillator_params(i, 0, 0, stand_offset); // Freq=0, Amp=0
        
    }
    // No coupling needed
    memset(coupling_weights, 0, sizeof(coupling_weights));
    memset(phase_offsets, 0, sizeof(phase_offsets));
}

// Call this to set the TROT gait
void set_gait_trot() {
    // --- 1. Reset all couplings ---
    memset(coupling_weights, 0, sizeof(coupling_weights));
    memset(phase_offsets, 0, sizeof(phase_offsets));

    // --- 2. Set Oscillator base parameters ---
    CPG_network_pram.base_freq = TWO_PI *0.7f; //0.2 1.0 Hz (1 cycle/sec)
    CPG_network_pram.hip_amp = 32704.0f;  // Example: 1500 counts swing
    CPG_network_pram.knee_amp = 65408.0f;   // Example: 800 counts lift
    CPG_network_pram.hip_offset = 0.0f;   //Hip center position
    CPG_network_pram.knee_offset = 0.0f; // Example: knee center position

    // set max_amp safely
     CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
     if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f; // fallback

    // Set params for all Hips
    set_oscillator_params(FLH, CPG_network_pram.base_freq, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, CPG_network_pram.base_freq, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, CPG_network_pram.base_freq, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, CPG_network_pram.base_freq, CPG_network_pram.hip_amp, CPG_network_pram.hip_offset);
    
    // Set params for all Knees
    set_oscillator_params(FLK, CPG_network_pram.base_freq, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, CPG_network_pram.base_freq, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, CPG_network_pram.base_freq, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, CPG_network_pram.base_freq, CPG_network_pram.knee_amp, CPG_network_pram.knee_offset);

    // --- 3. Define Coupling ---
    float K_intra_leg = 10.0f; // Strong coupling between a hip and its knee
    float K_inter_leg = 5.0f;  // Medium coupling between legs

    // === INTRA-LEG Coupling (Hip to Knee) ===
    // We want the knee to lift (e.g., sin(phi)=1) *before* the hip
    // moves forward (e.g., sin(phi)=0 -> 1).
    // A -90 degree (-PI/2) offset on the knee relative to the hip is common.
    //if(KH_offset==0){
     float  KH_offset =4.0f;
    //}

    float knee_lag = -TWO_PI / KH_offset;// Knee lags hip by 90 deg
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

    // Seed trot pattern with initial phases for instant equilibrium
// Group 1 (FL/BR): hips at 0
cpg_network[FLH].phase = 0.0f;
cpg_network[BRH].phase = 0.0f;
cpg_network[FLK].phase = 0.0f + (M_PI / 2.0f);  // Knee leads hip by 90° (from knee_lag = -PI/2)
cpg_network[BRK].phase = 0.0f + (M_PI / 2.0f);

// Group 2 (FR/BL): hips at PI (anti-phase to Group 1)
cpg_network[FRH].phase = M_PI;
cpg_network[BLH].phase = M_PI;
cpg_network[FRK].phase = M_PI + (M_PI / 2.0f);  // Wrap if needed: fmod(1.5*PI, 2*PI)
cpg_network[BLK].phase = M_PI + (M_PI / 2.0f);

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

        mode = 1;//get_mode();

        if (mode != last_mode) { // Only update when the mode *changes*
            ESP_LOGI(TAG, "Mode change detected: %d", mode);
            
            // --- Take the Mutex ---
          if (xSemaphoreTake(cpg_params_mutex, 100) == pdTRUE) {

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
        
         // --- Give the Mutex Back ---
         xSemaphoreGive(cpg_params_mutex);

         } else {
                ESP_LOGE(TAG, "Seq. task failed to get mutex!");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Check for a new mode 10 times/sec
    }
}

void hardware_Timer_setup(void){
    // Setup high-resolution timer for 10 kHz
    esp_timer_create_args_t timer_args = {
        .callback = &cpg_timer_callback,  // Function to call
        .arg = NULL,                      // Argument passed to callback
        .dispatch_method = ESP_TIMER_TASK, // How to dispatch callback
        .name = "cpg_timer"               // Timer name for debugging
    };

    if (esp_timer_create(&timer_args, &cpg_pid_timer_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CPG timer!");
        return;
    }

    // Start the 10 kHz timer
    if (esp_timer_start_periodic(cpg_pid_timer_handle, PID_UPDATE_RATE_US) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start CPG timer!");
        return;
    }
}

void CPG_app_main(void) {

    ESP_LOGI(TAG, "settingup the motor comnads");

    cpg_params_mutex = xSemaphoreCreateMutex(); 
    if (cpg_params_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create CPG mutex!");
    }

    // Create queue for timer notifications
    cpg_update_queue = xQueueCreate(1000, sizeof(cpg_update_cmd_t));
    if (cpg_update_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CPG update queue!");
        return;
    }

    // One-time phase initialization 
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
       cpg_network[i].phase = 0.0f;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // --- NEW CPG Setup ---
    ESP_LOGI(TAG, "Initializing CPG network...");
    // Initialize to a safe state
    set_gait_idle(); 
    // ---

    static TaskHandle_t seq_runner_handle = NULL;

   xTaskCreatePinnedToCore(sequence_runner_task, 
                            "seq_runner",
                             4096, 
                             NULL, 
                             9, 
                             &seq_runner_handle, 
                             1);

    vTaskDelay(pdMS_TO_TICKS(100));

    // --- CPG Task ---
    xTaskCreatePinnedToCore(cpg_update_task, 
                            "cpg_updater", 
                            4096, // CPG task might need stack for float math
                            NULL, 
                            19, // Higher priority than sequence runner
                            &cpg_task_handle, 
                            0); // Pin to core 1
    // ---
    vTaskDelay(pdMS_TO_TICKS(100));

    hardware_Timer_setup();
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "settingup the motor comnads complete");
}