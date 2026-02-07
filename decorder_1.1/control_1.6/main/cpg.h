#pragma once

#include "control_1.6.h" 
#include "pid.h"

#define NUM_OSCILLATORS NUM_MOTORS // 8 oscillators for 8 motors
#define CMD_BUFFER_SIZE 2 

// Define motor indices to make life easier
// Assuming 2 motors/leg: Hip (forward/back) and Knee (up/down)
#define FLH 0 // Front-Left Hip
#define FLK 1 // Front-Left Knee
#define FRH 2 // Front-Right Hip
#define FRK 3 // Front-Right Knee
#define BLH 4 // Back-Left Hip
#define BLK 5 // Back-Left Knee
#define BRH 6 // Back-Right Hip
#define BRK 7 // Back-Right knee_amp                                

typedef struct {
    float phase;        // Current phase (0 to 2*PI)
    float omega;        // Natural angular frequency (2*PI*f)
    float amplitude;    // Output amplitude (e.g., 1000 encoder counts)
    float offset;       // Output offset (center position)
    float output;       // The final calculated setpoint
} Oscillator;

typedef struct{
 float base_freq;
 float hip_amp ;
 float knee_amp;
 float hip_offset; 
 float knee_offset;
 float KH_offset;
 float max_amp;   
}osc_pram;

typedef enum {
    MODE_IDLE = 0,
    MODE_CRAWL,
    MODE_STANDBY,
    MODE_TURTLE
} SequenceMode;

// The global CPG network
extern Oscillator cpg_network[NUM_OSCILLATORS];
extern motor_Command_t cmd_buffer[CMD_BUFFER_SIZE][NUM_MOTORS];
extern volatile uint8_t write_index ;
extern volatile uint8_t read_index ;
extern portMUX_TYPE cpg_ring_mux ;

// Global gait parameter matrices
// These define the "brain" of the gait
extern float coupling_weights[NUM_OSCILLATORS][NUM_OSCILLATORS];
extern float phase_offsets[NUM_OSCILLATORS][NUM_OSCILLATORS];
void CPG_app_main(void);
void Update_Oscillator_base_parameters(void);