#pragma once

#include "control_1.8.h"  // Assuming this is the main control header

// ====================== Configuration Constants ======================
// Timing (easy to tune at top)
#define CPG_UPDATE_RATE_HZ      500.0f  // Hz (control loop frequency)
#define CPG_DT                  (1.0f / CPG_UPDATE_RATE_HZ)  // Seconds
#define CPG_UPDATE_RATE_US      (CPG_DT * 1000000.0f)  // Microseconds
#define PID_DT                  CPG_DT  // PID sample time (sync with CPG)

// Motor/PWM Limits (easy to adjust)
#define NUM_OSCILLATORS         NUM_MOTORS  // One oscillator per motor
#define PWM_MAX                 1023.0f  // Full-scale PWM duty
#define PWM_MIN                 900.0f   // Minimum duty to initiate motion
#define PWM_STOP                0.0f     // Below this: coast/stop
#define MIN_DUTY                (PWM_MIN * 1.0f)  // 60% of min for safety
#define COUNTS_PER_REV          28       // Encoder resolution per revolution
#define STOP_THRESHOLD          20       // "Stopped" threshold (ticks)
#define TOL                     (STOP_THRESHOLD*5)     // Position tolerance (encoder ticks)


// Debug/Logging Toggles
#define ENABLE_DEBUG_LOGS       1  // Global debug enable
#define ENABLE_PID_DEBUG        0
#define ENABLE_CPG_DEBUG        0
#define ENABLE_MOTOR_DEBUG      0
#define ENABLE_UART_DEBUG       0

// Logging Tags
#define TAG_PID     "PID_CTRL"
#define TAG_CPG     "CPG_NET"
#define TAG_MOTOR   "MOTOR_CMD"
#define TAG_SEQ     "SEQ_RUN"
#define TAG_UART    "UART_SEND"

// Math Constants
#define TWO_PI      (2.0f * 3.1415926535f)

#define CPG_frequency       (0.090f)
#define CPG_creep_frequency (0.20f)

#define KNEE_TURN_MOD_FACTOR 0.0f  // 0% modulation; tune 0.2-0.4 based on robot mass/inertia
#define HIP_TURN_MOD_FACTOR 0.6f  // 30% modulation; tune 0.2-0.4 based on robot mass/inertia



// ====================== Motor Indices (for clarity) ======================
#define FLH 0  // Front-Left Hip
#define FLK 1  // Front-Left Knee
#define FRH 2  // Front-Right Hip
#define FRK 3  // Front-Right Knee
#define BLH 4  // Back-Left Hip
#define BLK 5  // Back-Left Knee
#define BRH 6  // Back-Right Hip
#define BRK 7  // Back-Right Knee

/*#define FLH   // Front-Left Hip
#define FLK 3  // Front-Left Knee
#define FRH 4  // Front-Right Hip
#define FRK 5  // Front-Right Knee
#define BLH 2  // Back-Left Hip
#define BLK 1  // Back-Left Knee
#define BRH 6  // Back-Right Hip
#define BRK 7  // Back-Right Knee  */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

// ====================== Data Structures ======================
// PID Controller (position control)
typedef struct {
    float Kp, Ki, Kd;           // Gains (tunable)
    float T;                    // Sample time (auto-set)
    float limMin, limMax;       // Output limits
    float limMinInt, limMaxInt; // Integrator limits
    float proportional;         // P term
    float differentiator;       // D term
    float integrator;           // I state
    float error, prevError;     // Error tracking
    float prevMeasurement;      // For derivative
    float output, lastOutput;   // Output with smoothing
} PIDController;

// Motor State (per motor)
typedef struct {
    uint8_t m_num;              // Motor ID (1-based)
    float target_position;      // Current setpoint
    float prev_target;          // Previous for velocity calc
    int current_position;       // Filtered encoder reading
    int last_enc_val;           // Previous raw for stall/vel
    bool active;                // Enabled flag
    PIDController pos_pid;      // PID instance
} Motor;

// Motor Command Packet (for queueing)
typedef struct {
    float motor_target[NUM_MOTORS];  // Targets for all motors
    bool activate;                   // Global enable
} MotorCommand_t;

// PID Debug Log (for logging/telemetry)
typedef struct {
    float error;
    float proportional;
    float derivative;
    float integrator;
    int32_t encoder_val;
} PIDDebugLog_t;

// Gait Modes (easy to extend)
typedef enum {  
    MODE_CRAWL =3 ,
    MODE_TURTLE  ,
    MODE_CREEP   
} SequenceMode;

typedef enum {
    CPG_MODE_IDLE = 0,     // CPG stopped, motors off
    CPG_MODE_STANDBY,      // CPG running, zero amplitude, posture hold
    CPG_MODE_ACTIVE        // Normal gait
} CPG_RunMode;

typedef enum {
    LEG_ORIENTATION_NORMAL = 6,      // Robot upright
    LEG_ORIENTATION_INVERTED         // Robot upside down (legs flipped)
} LegOrientation;

typedef enum {
    MODE_TROT_LEFT=8,
    MODE_TROT_RIGHT, 
    MODE_CREEP_LEFT ,
    MODE_CREEP_RIGHT,
    MODE_CRAWL_LEFT,
    MODE_CRAWL_RIGHT,
    MODE_PIVOT_TURN,
    MODE_NORMAL, 
} turning_modes;

typedef enum {
    BODY_POSTURE_NORMAL =16,
    BODY_POSTURE_LOW,
    BODY_POSTURE_CROUCH
} body_posture_t;

typedef enum {
    STRAIGHT = 0,
    LEFT,
    RIGHT
} turn_mode_t;

// CPG Oscillator (per joint)
typedef struct {
    float phase;     // Current phase [0, 2*PI)
    float omega;     // Angular frequency (rad/s)
    float amplitude; // Output swing (± counts)
    float offset;    // Center position
    float output;    // Computed setpoint
} Oscillator;

// CPG Parameters (gait tuning)
typedef struct{
    float base_freq;
    float hip_amp;
    float knee_amp;
    float hip_amp_left;
    float hip_amp_right;
    float knee_amp_left;  
    float knee_amp_right;
    float hip_offset;
    float knee_offset;
    float KH_offset;
    float max_amp;
    float knee_omega_mult; // Default 2.0 (knees 2x hips)
    float hip_omega_mult; // Default 1.0 (base)
} osc_pram;

// ====================== Globals (Externals) ======================
// PID/Motor Globals
extern Motor motors[NUM_MOTORS];  // Main motor array
//extern Motor motor_log[NUM_MOTORS];  // Debug copy?
//extern QueueHandle_t motor_command_queue;  // Command queue

// Double Buffer for Safe Command Passing (PID outputs to UART)
extern volatile  float motor_cmd_buffer[2][NUM_MOTORS];
extern volatile  uint8_t current_write_index;  // Active write buffer
extern portMUX_TYPE pid_buffer_swap_mux;  // Swap protection

// CPG Globals
extern volatile Oscillator cpg_network[NUM_OSCILLATORS];  // Oscillator array
extern volatile osc_pram CPG_network_pram;                // Tunable params
extern volatile float coupling_weights[NUM_OSCILLATORS][NUM_OSCILLATORS];  // Coupling matrix
extern volatile float phase_offsets[NUM_OSCILLATORS][NUM_OSCILLATORS];    // Phase offset matrix
extern volatile uint8_t current_mode;  // Active gait mode
extern SemaphoreHandle_t cpg_params_mutex;
extern volatile CPG_RunMode cpg_run_mode;


// ====================== Function Declarations ======================
// PID Initialization and Main
void PIDController_Init(PIDController *pid);
void update_PID_gain(void);
void PIDController_Update(PIDController *pid, float setpoint, int measurement, uint8_t i);
void pid_init_all(void);  // Initialize motors and PIDs
void pid_app_main(void);  // Entry point: create PID task
void run_position_loop(void);  // Main PID loop

// UART Sending (for motor outputs)
void update_uart_motor_commands(float *new_commands);  // Atomic double-buffer write and UART send

// CPG Initialization and Main
void cpg_init_all(void);  // Initialize oscillators and params
void CPG_app_main(void);  // Entry point: setup CPG tasks/timer
void Update_Oscillator_base_parameters(void);  // Update params from globals
void set_oscillator_params(int i, float omega, float amp, float offset);

// Utility: Safe command update
void motor_set(uint8_t i, float target, bool set);  // Set motor target

// Debug: Log PID state
void log_pid_debug(uint8_t motor_idx, PIDDebugLog_t *log);