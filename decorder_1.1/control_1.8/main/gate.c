#include "cpg.h"
#include "gate.h"

#define SMOOTH_ALPHA_INIT 1.0f  // First tick: blend fully from current to target for smooth start
#define GAIT_TROT  0
#define GAIT_CREEP 1
#define GAIT_CRAWL 2
#define GAIT_WALK  3
#define GAIT_GALOP 4
static uint8_t s_last_gait_type = 0xFF;  // Only reseed phases when gait type changes

static inline void set_gait_trot_left(void);
static inline void set_gait_trot_right(void);
static inline void set_gait_crawl_left(void);
static inline void set_gait_crawl_right(void);
static inline void set_gait_creep_left(void);
static inline void set_gait_creep_right(void);
static inline void set_gait_walk_left(void);
static inline void set_gait_walk_right(void);
static inline void set_gait_galop_left(void);
static inline void set_gait_galop_right(void); 

// Gait: IDLE (motors off, output 0)
inline void set_gait_idle(void)
{
    cpg_run_mode = CPG_MODE_IDLE;
    s_last_gait_type = 0xFF;
    CPG_network_pram.diagonal_knee_boost = 1.0f;
    memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    memset((void*)phase_offsets, 0, sizeof(phase_offsets));
}

// Gait: HOMING (drive all motors to position 0)
inline void set_gait_homing(void)
{
    cpg_run_mode = CPG_MODE_HOMING;
    s_last_gait_type = 0xFF;
    CPG_network_pram.diagonal_knee_boost = 1.0f;
    memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    memset((void*)phase_offsets, 0, sizeof(phase_offsets));
}

// Gait: CRAWL (placeholder)
inline void set_gait_crawl(uint8_t func_mode ,uint8_t posture ) {

    cpg_run_mode = CPG_MODE_ACTIVE;

    if(func_mode == STRAIGHT){
     memset((void*)coupling_weights, 0, sizeof(coupling_weights));
     memset((void*)phase_offsets, 0, sizeof(phase_offsets));
    }

    CPG_network_pram.duty_cycle = 0.50f;
    CPG_network_pram.damping = 0.10f;
    CPG_network_pram.diagonal_knee_boost = 1.0f;

    CPG_network_pram.base_freq = TWO_PI * CPG_frequency;

    if(posture == BODY_POSTURE_NORMAL){
      CPG_network_pram.hip_amp = 16000.0f;
      CPG_network_pram.knee_amp = 16000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = 0.0f;
    }
    else if(posture == BODY_POSTURE_LOW){
      CPG_network_pram.hip_amp = 12000.0f;
      CPG_network_pram.knee_amp = 12000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = 0.0f;
    }
    else {
      CPG_network_pram.hip_amp = 9000.0f;
      CPG_network_pram.knee_amp = 9000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = 0.0f;
    }

    CPG_network_pram.KH_offset = 4.0f;

    // FIXED: Milder scaling (10-15% max cut at 0.2 Hz; hips slightly more to ease inertia)
    float hip_freq_factor = fminf(1.0f / sqrtf(1.0f + 2.0f * CPG_frequency), 0.90f);  // Coeff 2.0f: ~12% cut
    float knee_freq_factor = fminf(1.0f / sqrtf(1.0f + 1.5f * CPG_frequency), 0.92f); // Coeff 1.5f: ~8% cut
    CPG_network_pram.hip_amp *= hip_freq_factor;
    CPG_network_pram.knee_amp *= knee_freq_factor;

    // FIXED: Lower mult for stability (1.5x knees = ~1.5 cycles; tune 1.2-1.8 to avoid pull)
    CPG_network_pram.hip_omega_mult = 1.2f;
    CPG_network_pram.knee_omega_mult = 1.5f;  // Subtle fast knees without distortion

   // Recompute max_amp
   CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
   if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

   if(func_mode == LEFT) set_gait_crawl_left();
   else if(func_mode == RIGHT) set_gait_crawl_right();
   else {
      CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp;
      CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
      CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp;
    }

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
    coupling_weights[FLH][FRH] = K_inter_leg; phase_offsets[FLH][FRH] = 0;
    coupling_weights[FRH][FLH] = K_inter_leg; phase_offsets[FRH][FLH] = 0;
    coupling_weights[FLH][BLH] = K_inter_leg; phase_offsets[FLH][BLH] = 0;
    coupling_weights[BLH][FLH] = K_inter_leg; phase_offsets[BLH][FLH] = 0;
    coupling_weights[FRH][BRH] = K_inter_leg; phase_offsets[FRH][BRH] = 0;
    coupling_weights[BRH][FRH] = K_inter_leg; phase_offsets[BRH][FRH] = 0;
    coupling_weights[BLH][BRH] = K_inter_leg; phase_offsets[BLH][BRH] = 0;
    coupling_weights[BRH][BLH] = K_inter_leg; phase_offsets[BRH][BLH] = 0;
    if (s_last_gait_type != GAIT_CRAWL) {
        cpg_network[FLH].phase = 0.0f; cpg_network[BRH].phase = 0.0f;
        cpg_network[FLK].phase = 0.0f; cpg_network[BRK].phase = 0.0f;
        cpg_network[FRH].phase = 0.0f; cpg_network[BLH].phase = 0.0f;
        cpg_network[FRK].phase = 0.0f; cpg_network[BLK].phase = 0.0f;
    }
    s_last_gait_type = GAIT_CRAWL;
}

// Gait: TROT (diagonal sync, alias for MODE_TURTLE)
inline void set_gait_trot(uint8_t func_mode,uint8_t posture) {
    cpg_run_mode = CPG_MODE_ACTIVE;

    if(func_mode == STRAIGHT){
     memset((void*)coupling_weights, 0, sizeof(coupling_weights));
     memset((void*)phase_offsets, 0, sizeof(phase_offsets));
    }

    // Set Duty Cycle for Trot ---
    CPG_network_pram.duty_cycle = 0.5f; // Symmetric swing/stance
    CPG_network_pram.damping = 0.10f;
    CPG_network_pram.diagonal_knee_boost = 1.0f;

    CPG_network_pram.base_freq = TWO_PI * CPG_frequency;

    if(posture == BODY_POSTURE_NORMAL){
      CPG_network_pram.hip_amp = 16352.0f;
      CPG_network_pram.knee_amp = 32704.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -23000.0f;  
    }
    else if(posture == BODY_POSTURE_LOW){
      CPG_network_pram.hip_amp = 16352.0f;
      CPG_network_pram.knee_amp = 32704.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -20000.0f;  
    }
    else {
      CPG_network_pram.hip_amp = 16352.0f;
      CPG_network_pram.knee_amp = 32704.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = 0.0f;  
    }
    
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

   if(func_mode == LEFT) set_gait_trot_left();
   else if(func_mode == RIGHT) set_gait_trot_right();
   else {
     CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp;
     CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp;
     CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp;
     CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp;
     CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
     CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp;
     CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp;
     CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp;
     CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp;
    }

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
    if (s_last_gait_type != GAIT_TROT) {
        cpg_network[FLH].phase = 0.0f; cpg_network[BRH].phase = 0.0f;
        cpg_network[FLK].phase = TWO_PI / 4.0f; cpg_network[BRK].phase = TWO_PI / 4.0f;
        cpg_network[FRH].phase = TWO_PI / 2.0f; cpg_network[BLH].phase = TWO_PI / 2.0f;
        cpg_network[FRK].phase = TWO_PI / 2.0f + TWO_PI / 4.0f; cpg_network[BLK].phase = TWO_PI / 2.0f + TWO_PI / 4.0f;
    }
    s_last_gait_type = GAIT_TROT;
}

// Sequence: BR -> FR -> BL -> FL
// Gait: CREEP (Lateral Sequence - 4 Beat)

inline void set_gait_creep(uint8_t func_mode,uint8_t posture) {
    cpg_run_mode = CPG_MODE_ACTIVE;

    if(func_mode == STRAIGHT){
     memset((void*)coupling_weights, 0, sizeof(coupling_weights));
     memset((void*)phase_offsets, 0, sizeof(phase_offsets));
    }

    
    // 0.75 ensures the "3-legs-down" rule for lateral sequence stability
    CPG_network_pram.duty_cycle = 0.80f;
    CPG_network_pram.damping = 0.1250f;
    /* Boost diagonal stance knee when opposite leg swings: tilts robot toward stride triangle for stability */
    CPG_network_pram.diagonal_knee_boost = 1.30f;

    CPG_network_pram.base_freq = TWO_PI * CPG_creep_frequency;
   
    // Amplitudes for long steps but controlled lift

    if(posture == BODY_POSTURE_NORMAL){
      CPG_network_pram.hip_amp = 8000.0f;
      CPG_network_pram.knee_amp = 17000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -15000.0f;
    }
    else if(posture == BODY_POSTURE_LOW){
      CPG_network_pram.hip_amp = 8000.0f;
      CPG_network_pram.knee_amp = 14000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -12000.0f;
    }
    else {
      CPG_network_pram.hip_amp = 15000.0f;
      CPG_network_pram.knee_amp = 6000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = 0.0f; 
    }
   
    CPG_network_pram.KH_offset = 8.0f;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
   
    // Set to 1:1 for strict synchronization
    CPG_network_pram.hip_omega_mult = 1.0f;
    CPG_network_pram.knee_omega_mult = 1.0f;
   
    if(func_mode == LEFT) set_gait_creep_left();
    else if(func_mode == RIGHT) set_gait_creep_right();
    else {
      CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp;

      CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
      
      CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp;
    }

    // ================== COUPLINGS ==================
    float K_intra_leg = 20.0f; // Keep tight hip-knee sync
    float K_inter_leg = 8.0f;  // Lowered slightly to prevent sudden pulling
    float knee_lag = -TWO_PI / CPG_network_pram.KH_offset;
    
    // Each leg lags the previous by 90° → correct sequence BR -> FR -> BL -> FL
    float offset_lag = TWO_PI / 4.0f;

    // 1. Intra-leg (Hip -> Knee) - Unchanged
    coupling_weights[FLK][FLH] = K_intra_leg; phase_offsets[FLK][FLH] = knee_lag;
    coupling_weights[FRK][FRH] = K_intra_leg; phase_offsets[FRK][FRH] = knee_lag;
    coupling_weights[BLK][BLH] = K_intra_leg; phase_offsets[BLK][BLH] = knee_lag;
    coupling_weights[BRK][BRH] = K_intra_leg; phase_offsets[BRK][BRH] = knee_lag;

    // 2. Inter-leg RING (Strict Sequence: BR -> FR -> BL -> FL -> BR)
    coupling_weights[FRH][BRH] = K_inter_leg; phase_offsets[FRH][BRH] = offset_lag;
    coupling_weights[BLH][FRH] = K_inter_leg; phase_offsets[BLH][FRH] = offset_lag;
    coupling_weights[FLH][BLH] = K_inter_leg; phase_offsets[FLH][BLH] = offset_lag;
    coupling_weights[BRH][FLH] = K_inter_leg; phase_offsets[BRH][FLH] = offset_lag;

    if (s_last_gait_type != GAIT_CREEP) {
        // BR=0 (swing), FR=270°, BL=180°, FL=90° → sequence BR->FR->BL->FL
        cpg_network[BRH].phase = 0.0f;
        cpg_network[BRK].phase = 0.0f + knee_lag;
        cpg_network[FRH].phase = TWO_PI * 0.75f;
        cpg_network[FRK].phase = (TWO_PI * 0.75f) + knee_lag;
        cpg_network[BLH].phase = TWO_PI * 0.50f;
        cpg_network[BLK].phase = (TWO_PI * 0.50f) + knee_lag;
        cpg_network[FLH].phase = TWO_PI * 0.25f;
        cpg_network[FLK].phase = (TWO_PI * 0.25f) + knee_lag;
        for(int i=0; i<NUM_OSCILLATORS; i++) {
            while(cpg_network[i].phase < 0) cpg_network[i].phase += TWO_PI;
            cpg_network[i].phase = fmodf(cpg_network[i].phase, TWO_PI);
        }
    }
    s_last_gait_type = GAIT_CREEP;
}

// Sequence: BR -> FL -> BL -> FR (Diagonal sequence - 4 beat, staggered like creep)
// Gait: WALK - same structure as creep but diagonal order (each leg 90° apart)

inline void set_gait_walk(uint8_t func_mode,uint8_t posture) {
    cpg_run_mode = CPG_MODE_ACTIVE;

    if(func_mode == STRAIGHT){
     memset((void*)coupling_weights, 0, sizeof(coupling_weights));
     memset((void*)phase_offsets, 0, sizeof(phase_offsets));
    }

    CPG_network_pram.duty_cycle = 0.85f;   // 80% stance for 3-leg support (like creep)
    CPG_network_pram.damping = 0.125f;
    CPG_network_pram.diagonal_knee_boost = 1.30f;
    CPG_network_pram.base_freq = TWO_PI * CPG_creep_frequency;

    if(posture == BODY_POSTURE_NORMAL){
      CPG_network_pram.hip_amp = 6000.0f;
      CPG_network_pram.knee_amp = 16500.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -15000.0f;
    }
    else if(posture == BODY_POSTURE_LOW){
      CPG_network_pram.hip_amp = 8000.0f;
      CPG_network_pram.knee_amp = 14000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -12000.0f;
    }
    else {
      CPG_network_pram.hip_amp = 7000.0f;
      CPG_network_pram.knee_amp = 12000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -10000.0f;
    }

    CPG_network_pram.KH_offset = 8.0f;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    CPG_network_pram.hip_omega_mult = 1.0f;
    CPG_network_pram.knee_omega_mult = 1.0f;

    if(func_mode == LEFT) set_gait_walk_left();
    else if(func_mode == RIGHT) set_gait_walk_right();
    else {
      CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp;
      CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
      CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp;
    }

    // Diagonal sequence RING: BR -> FL -> BL -> FR -> BR (each leg 90° apart, staggered like creep)
    float K_intra_leg = 20.0f;
    float K_inter_leg = 8.0f;
    float knee_lag = -TWO_PI / CPG_network_pram.KH_offset;
    float offset_lag = TWO_PI / 4.0f;  // 90° between legs

    coupling_weights[FLK][FLH] = K_intra_leg; phase_offsets[FLK][FLH] = knee_lag;
    coupling_weights[FRK][FRH] = K_intra_leg; phase_offsets[FRK][FRH] = knee_lag;
    coupling_weights[BLK][BLH] = K_intra_leg; phase_offsets[BLK][BLH] = knee_lag;
    coupling_weights[BRK][BRH] = K_intra_leg; phase_offsets[BRK][BRH] = knee_lag;

    coupling_weights[FLH][BRH] = K_inter_leg; phase_offsets[FLH][BRH] = offset_lag;  // FL lags BR
    coupling_weights[BLH][FLH] = K_inter_leg; phase_offsets[BLH][FLH] = offset_lag;  // BL lags FL
    coupling_weights[FRH][BLH] = K_inter_leg; phase_offsets[FRH][BLH] = offset_lag;  // FR lags BL
    coupling_weights[BRH][FRH] = K_inter_leg; phase_offsets[BRH][FRH] = offset_lag;  // BR lags FR

    if (s_last_gait_type != GAIT_WALK) {
        // BR=0, FL=90°, BL=180°, FR=270° → sequence BR->FL->BL->FR
        cpg_network[BRH].phase = 0.0f;
        cpg_network[BRK].phase = 0.0f + knee_lag;
        cpg_network[FLH].phase = TWO_PI * 0.25f;
        cpg_network[FLK].phase = (TWO_PI * 0.25f) + knee_lag;
        cpg_network[BLH].phase = TWO_PI * 0.50f;
        cpg_network[BLK].phase = (TWO_PI * 0.50f) + knee_lag;
        cpg_network[FRH].phase = TWO_PI * 0.75f;
        cpg_network[FRK].phase = (TWO_PI * 0.75f) + knee_lag;
        for(int i=0; i<NUM_OSCILLATORS; i++) {
            while(cpg_network[i].phase < 0) cpg_network[i].phase += TWO_PI;
            cpg_network[i].phase = fmodf(cpg_network[i].phase, TWO_PI);
        }
    }
    s_last_gait_type = GAIT_WALK;
}

// Gait: GALLOP (Bound - rear pair then front pair)
// BR+BL in phase, FR+FL in phase, 180° between rear and front

inline void set_gait_galop(uint8_t func_mode,uint8_t posture) {
    cpg_run_mode = CPG_MODE_ACTIVE;

    if(func_mode == STRAIGHT){
     memset((void*)coupling_weights, 0, sizeof(coupling_weights));
     memset((void*)phase_offsets, 0, sizeof(phase_offsets));
    }

    CPG_network_pram.duty_cycle = 0.55f;   // Shorter stance for bound gait
    CPG_network_pram.damping = 0.15f;     // Stronger coupling for synced pairs
    CPG_network_pram.diagonal_knee_boost = 1.0f;
    CPG_network_pram.base_freq = TWO_PI * CPG_frequency;  // Slightly faster than creep

    if(posture == BODY_POSTURE_NORMAL){
      CPG_network_pram.hip_amp = 12000.0f;
      CPG_network_pram.knee_amp = 22000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -20000.0f;
    }
    else if(posture == BODY_POSTURE_LOW){
      CPG_network_pram.hip_amp = 10000.0f;
      CPG_network_pram.knee_amp = 18000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -17000.0f;
    }
    else {
      CPG_network_pram.hip_amp = 8000.0f;
      CPG_network_pram.knee_amp = 14000.0f;
      CPG_network_pram.hip_offset = 0.0f;
      CPG_network_pram.knee_offset = -12000.0f;
    }

    CPG_network_pram.KH_offset = 5.0f;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    CPG_network_pram.hip_omega_mult = 1.2f;
    CPG_network_pram.knee_omega_mult = 1.5f;

    if(func_mode == LEFT) set_gait_galop_left();
    else if(func_mode == RIGHT) set_gait_galop_right();
    else {
      CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp;
      CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
      CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp;
      CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp;
      CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp;
      CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp;
    }

    // Bound: BR+BL in phase, FR+FL in phase, 180° between rear and front
    float K_intra_leg = 15.0f;
    float K_inter_leg = 10.0f;
    float knee_lag = -TWO_PI / CPG_network_pram.KH_offset;

    coupling_weights[FLK][FLH] = K_intra_leg; phase_offsets[FLK][FLH] = knee_lag;
    coupling_weights[FRK][FRH] = K_intra_leg; phase_offsets[FRK][FRH] = knee_lag;
    coupling_weights[BLK][BLH] = K_intra_leg; phase_offsets[BLK][BLH] = knee_lag;
    coupling_weights[BRK][BRH] = K_intra_leg; phase_offsets[BRK][BRH] = knee_lag;

    coupling_weights[BRH][BLH] = K_inter_leg; phase_offsets[BRH][BLH] = 0;
    coupling_weights[BLH][BRH] = K_inter_leg; phase_offsets[BLH][BRH] = 0;
    coupling_weights[FRH][FLH] = K_inter_leg; phase_offsets[FRH][FLH] = 0;
    coupling_weights[FLH][FRH] = K_inter_leg; phase_offsets[FLH][FRH] = 0;
    coupling_weights[BRH][FRH] = K_inter_leg; phase_offsets[BRH][FRH] = TWO_PI / 2.0f;
    coupling_weights[FRH][BRH] = K_inter_leg; phase_offsets[FRH][BRH] = TWO_PI / 2.0f;
    coupling_weights[BLH][FLH] = K_inter_leg; phase_offsets[BLH][FLH] = TWO_PI / 2.0f;
    coupling_weights[FLH][BLH] = K_inter_leg; phase_offsets[FLH][BLH] = TWO_PI / 2.0f;

    if (s_last_gait_type != GAIT_GALOP) {
        cpg_network[BRH].phase = 0.0f;
        cpg_network[BRK].phase = 0.0f + knee_lag;
        cpg_network[BLH].phase = 0.0f;
        cpg_network[BLK].phase = 0.0f + knee_lag;
        cpg_network[FRH].phase = TWO_PI / 2.0f;
        cpg_network[FRK].phase = TWO_PI / 2.0f + knee_lag;
        cpg_network[FLH].phase = TWO_PI / 2.0f;
        cpg_network[FLK].phase = TWO_PI / 2.0f + knee_lag;
        for(int i=0; i<NUM_OSCILLATORS; i++) {
            while(cpg_network[i].phase < 0) cpg_network[i].phase += TWO_PI;
            cpg_network[i].phase = fmodf(cpg_network[i].phase, TWO_PI);
        }
    }
    s_last_gait_type = GAIT_GALOP;
}

// Gait: STANDBY (hold)
void set_gait_standby(void)
{
    cpg_run_mode = CPG_MODE_STANDBY;
    CPG_network_pram.diagonal_knee_boost = 1.0f;

    memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    memset((void*)phase_offsets, 0, sizeof(phase_offsets));

}

 
 //******************************** Turning Gates ****************************************************
//****************************************************************************************************

//Left-turning trot (symmetric to left)

static inline void set_gait_trot_left(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right_target, CPG_network_pram.knee_amp_right_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

// New: Right-turning trot (symmetric to left)
static inline void set_gait_trot_right(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left_target, CPG_network_pram.knee_amp_left_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}


//Left-turning gait_creep (symmetric to left) 

static inline void set_gait_creep_left(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right_target, CPG_network_pram.knee_amp_right_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

//  Right-turning trot (symmetric to left)
static inline void set_gait_creep_right(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left_target, CPG_network_pram.knee_amp_left_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

//Left-turning crawl (symmetric to left)

static inline void set_gait_crawl_left(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right_target, CPG_network_pram.knee_amp_right_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

static inline void set_gait_crawl_right(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left_target, CPG_network_pram.knee_amp_left_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

static inline void set_gait_walk_left(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right_target, CPG_network_pram.knee_amp_right_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

static inline void set_gait_walk_right(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left_target, CPG_network_pram.knee_amp_left_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

static inline void set_gait_galop_left(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right_target, CPG_network_pram.knee_amp_right_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}

static inline void set_gait_galop_right(void) {
    CPG_network_pram.hip_amp_left_target = CPG_network_pram.hip_amp * (1.0f + 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right_target = CPG_network_pram.hip_amp * (1.0f - 0.5f * HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left_target = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right_target = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.duty_cycle_target = CPG_network_pram.duty_cycle;
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left_target, CPG_network_pram.knee_amp_left_target);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;
}


