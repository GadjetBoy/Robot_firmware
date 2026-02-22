#include "cpg.h"
#include "gate.h"

static inline void set_gait_trot_left(void);
static inline void set_gait_trot_right(void);
static inline void set_gait_crawl_left(void);
static inline void set_gait_crawl_right(void);
static inline void set_gait_creep_left(void);
static inline void set_gait_creep_right(void); 

// Gait: IDLE (all off)
inline void set_gait_idle(void)
{
    cpg_run_mode = CPG_MODE_IDLE;

    //memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    //memset((void*)phase_offsets, 0, sizeof(phase_offsets));

}


// Gait: TROT (diagonal sync, alias for MODE_TURTLE)
inline void set_gait_trot(uint8_t func_mode) {

    cpg_run_mode = CPG_MODE_ACTIVE;
    memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    memset((void*)phase_offsets, 0, sizeof(phase_offsets));

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

   if(func_mode == LEFT) set_gait_trot_left();
   else if(func_mode == RIGHT) set_gait_trot_right();
   else{

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
    // Seed phases
    cpg_network[FLH].phase = 0.0f; cpg_network[BRH].phase = 0.0f;
    cpg_network[FLK].phase = TWO_PI / 4.0f; cpg_network[BRK].phase = TWO_PI / 4.0f;
    cpg_network[FRH].phase = TWO_PI / 2.0f; cpg_network[BLH].phase = TWO_PI / 2.0f;
    cpg_network[FRK].phase = TWO_PI / 2.0f + TWO_PI / 4.0f; cpg_network[BLK].phase = TWO_PI / 2.0f + TWO_PI / 4.0f;
    
    //ESP_LOGI(TAG_CPG, "Gait set to TROT at freq=%.2f Hz", CPG_frequency);
}

// Gait: CRAWL (Creep Gait - 4 Beat Lateral Sequence)
// Sequence: BR -> FR -> BL -> FL
inline void set_gait_creep(uint8_t func_mode) {

    cpg_run_mode = CPG_MODE_ACTIVE;
    memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    memset((void*)phase_offsets, 0, sizeof(phase_offsets));

    CPG_network_pram.base_freq = TWO_PI * CPG_creep_frequency; 
    
    // For creep/crawl, we usually want high hip amplitude (long step) 
    // but moderate knee amplitude (just enough to clear ground).
    CPG_network_pram.hip_amp = 9000.0f;  
    CPG_network_pram.knee_amp = 11500.0f; // Slightly lower than trot to reduce impact
    CPG_network_pram.hip_offset = 0.0f;
    CPG_network_pram.knee_offset = -16000.0f;
    
    // KH_offset determines the phase shift between Hip and Knee.
    // 4.0 means 90 degrees (Quarter cycle lag), ensuring the leg lifts as it swings forward.
    CPG_network_pram.KH_offset = 4.0f; 

    // FIXED: Removed dynamic freq scaling used in trot. 
    // Creep is a low-speed, high-torque gait. We want consistent amplitude.
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Multipliers: Set to 1.0 for Walk. 
    // In Creep, Hip and Knee must remain perfectly synchronized (1:1) to ensure the foot 
    // lands exactly when the hip stride finishes.
    CPG_network_pram.hip_omega_mult = 1.5f;
    CPG_network_pram.knee_omega_mult = 1.0f;

    if(func_mode == LEFT) set_gait_creep_left();
    else if(func_mode == RIGHT) set_gait_creep_right();
    else{
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
    }

    // ================== COUPLINGS ==================
    float K_intra_leg = 10.0f; // Strong local coupling
    float K_inter_leg = 8.0f;  // Moderate gait coordination
    float knee_lag = -TWO_PI / CPG_network_pram.KH_offset; // Knee lags Hip by 90 deg

    // 1. Intra-leg (Hip -> Knee) - Same for all legs
    coupling_weights[FLK][FLH] = K_intra_leg; phase_offsets[FLK][FLH] = knee_lag;
    coupling_weights[FRK][FRH] = K_intra_leg; phase_offsets[FRK][FRH] = knee_lag;
    coupling_weights[BLK][BLH] = K_intra_leg; phase_offsets[BLK][BLH] = knee_lag;
    coupling_weights[BRK][BRH] = K_intra_leg; phase_offsets[BRK][BRH] = knee_lag;

    // 2. Inter-leg (The Gait Pattern)
    // We define a "Ring" of influence for stability: BR -> FR -> BL -> FL -> BR
    // Phase shift is 0.25 (90 degrees or TWO_PI/4) between steps.
    float quarter_cycle = TWO_PI / 4.0f;

    // Right Side: Back Right drives Front Right (FR lags BR by 90)
    coupling_weights[FRH][BRH] = K_inter_leg; phase_offsets[FRH][BRH] = quarter_cycle;
    coupling_weights[BRH][FRH] = K_inter_leg; phase_offsets[BRH][FRH] = -quarter_cycle;

    // Right Side: Back Right drives Front Right (FL lags BL by 90)
    coupling_weights[FLH][BLH] = K_inter_leg; phase_offsets[FLH][BLH] = quarter_cycle;
    coupling_weights[BLH][FLH] = K_inter_leg; phase_offsets[BLH][FLH] = -quarter_cycle;  // Added reciprocal

    // Front: Front Right drives Front Left (FL lags FR by 180 in total time, but we chain it)
    // Note: In lateral seq, FL lifts AFTER BL. 
    // Let's enforce the specific phase map directly between neighbors.
    
    // Contralateral (Left vs Right) neighbors usually 180 out of phase in pairs? 
    // No, for creep: BR(0) -> FR(0.25) -> BL(0.5) -> FL(0.75).
    
    // Connect Back Legs (BR -> BL is 180 shift)
    coupling_weights[BLH][BRH] = K_inter_leg; phase_offsets[BLH][BRH] = TWO_PI / 2.0f;
    coupling_weights[BRH][BLH] = K_inter_leg; phase_offsets[BRH][BLH] = - (TWO_PI / 2.0f);  // Fixed sign

    // Connect Front Legs (FR -> FL is 180 shift)
    coupling_weights[FLH][FRH] = K_inter_leg; phase_offsets[FLH][FRH] = TWO_PI / 2.0f;
    coupling_weights[FRH][FLH] = K_inter_leg; phase_offsets[FRH][FLH] = - (TWO_PI / 2.0f);  // Fixed sign

    // Connect Ipsilateral (Same side)

    float three_quarter_cycle = (TWO_PI / 4.0f)*3.0f;

    // FR <-> BL (270° offset: BL lags FR)
    coupling_weights[BLH][FRH] = K_inter_leg; phase_offsets[BLH][FRH] =  three_quarter_cycle;  // BL lags FR by 270°
    coupling_weights[FRH][BLH] = K_inter_leg; phase_offsets[FRH][BLH] = -three_quarter_cycle;  // FR leads BL by 270°

    // FL <-> BR (90° offset: BR lags FL)
    coupling_weights[BRH][FLH] = K_inter_leg; phase_offsets[BRH][FLH] =  quarter_cycle;  // BR lags FL by 90°
    coupling_weights[FLH][BRH] = K_inter_leg; phase_offsets[FLH][BRH] = -quarter_cycle;  // FL leads BR by 90°


    // ================== SEED PHASES ==================
    // Instantaneously set phases to the ideal distribution to minimize convergence time.
    // Map: BR=0, FR=0.25, BL=0.5, FL=0.75
    
    cpg_network[BRH].phase = 0.0f;
    cpg_network[BRK].phase = 0.0f + knee_lag; // Make sure to wrap in update task if < 0

    cpg_network[FRH].phase = TWO_PI * 0.25f;
    cpg_network[FRK].phase = (TWO_PI * 0.25f) + knee_lag;

    cpg_network[BLH].phase = TWO_PI * 0.50f;
    cpg_network[BLK].phase = (TWO_PI * 0.50f) + knee_lag;

    cpg_network[FLH].phase = TWO_PI * 0.75f;
    cpg_network[FLK].phase = (TWO_PI * 0.75f) + knee_lag;
    
    // Normalize phases to be positive (0 to 2PI)
    for(int i=0; i<NUM_OSCILLATORS; i++) {
        if(cpg_network[i].phase < 0) cpg_network[i].phase += TWO_PI;
        cpg_network[i].phase = fmodf(cpg_network[i].phase, TWO_PI);
    }
    
    //ESP_LOGI(TAG_CPG, "Gait set to CREEP (Lateral Seq) at freq=%.2f Hz", CPG_creep_frequency);
}

// Gait: CRAWL (placeholder)
inline void set_gait_crawl(uint8_t func_mode) {

    cpg_run_mode = CPG_MODE_ACTIVE;
    memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    memset((void*)phase_offsets, 0, sizeof(phase_offsets));

    CPG_network_pram.base_freq = TWO_PI * CPG_frequency;
    CPG_network_pram.hip_amp = 16000.0f;
    CPG_network_pram.knee_amp = 16000.0f;
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
    CPG_network_pram.knee_omega_mult = 1.5f;  // Subtle fast knees without distortion

   // Recompute max_amp
   CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp, CPG_network_pram.knee_amp);
   if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

   if(func_mode == LEFT) set_gait_crawl_left();
   else if(func_mode == RIGHT) set_gait_crawl_right();
   else{
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
    // Seed phases
    cpg_network[FLH].phase = 0.0f; cpg_network[BRH].phase = 0.0f;
    cpg_network[FLK].phase = 0; cpg_network[BRK].phase = 0;
    cpg_network[FRH].phase = 0; cpg_network[BLH].phase = 0;
    cpg_network[FRK].phase = 0; cpg_network[BLK].phase = 0;

    //ESP_LOGI(TAG_CPG, "Gait set to crawl at freq=%.2f Hz", CPG_frequency);
}

// Gait: STANDBY (hold)
void set_gait_standby(void)
{
    cpg_run_mode = CPG_MODE_STANDBY;

    memset((void*)coupling_weights, 0, sizeof(coupling_weights));
    memset((void*)phase_offsets, 0, sizeof(phase_offsets));

    CPG_network_pram.hip_offset  = 0.0f;
    CPG_network_pram.knee_offset = -16000.0f;

    for (int i = 0; i < NUM_OSCILLATORS; i++) {
        float offset = (i % 2 == 0)
            ? CPG_network_pram.hip_offset
            : CPG_network_pram.knee_offset;

        // KEEP omega, ZERO amplitude
        set_oscillator_params(
            i,
            cpg_network[i].omega,
            0.0f,
            offset
        );

    }
}

 
 //******************************** Turning Gates ****************************************************
//****************************************************************************************************

//Left-turning trot (symmetric to left)

static inline void set_gait_trot_left(void) {
    
    // Apply asymmetry
    CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp * (1.0f - 0.5f *HIP_TURN_MOD_FACTOR); // Milder for Hips
    CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp * (1.0f + 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);  
    CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);

    // Recompute max_amp (use the largest)
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right, CPG_network_pram.knee_amp_right);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Reset oscillators with asymmetric amps
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);

}

// New: Right-turning trot (symmetric to left)
static inline void set_gait_trot_right(void) {
    
    // Reverse asymmetry
    CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp * (1.0f + 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp * (1.0f - 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);

    // Recompute max_amp
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left, CPG_network_pram.knee_amp_left);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Reset oscillators (similar to left, but with swapped amps)
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);

}


//Left-turning gait_creep (symmetric to left) 

static inline void set_gait_creep_left(void) {

    // Apply asymmetry
    CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp * (1.0f - 0.5f *HIP_TURN_MOD_FACTOR); // Milder for Hips
    CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp * (1.0f + 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);  
    CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);

    // Recompute max_amp (use the largest)
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right, CPG_network_pram.knee_amp_right);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Reset oscillators with asymmetric amps
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);

}

//  Right-turning trot (symmetric to left)
static inline void set_gait_creep_right(void) {

    // Reverse asymmetry
    CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp * (1.0f + 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp * (1.0f - 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);

    // Recompute max_amp
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left, CPG_network_pram.knee_amp_left);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Reset oscillators (similar to left, but with swapped amps)
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);

}

//Left-turning crawl (symmetric to left)

static inline void set_gait_crawl_left(void) {
    
    // Apply asymmetry
    CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp * (1.0f - 0.5f *HIP_TURN_MOD_FACTOR); // Milder for Hips
    CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp * (1.0f + 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);  
    CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);

    // Recompute max_amp (use the largest)
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_right, CPG_network_pram.knee_amp_right);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Reset oscillators with asymmetric amps
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);

}

// New: Right-turning trot (symmetric to left)
static inline void set_gait_crawl_right(void) {
    
    // Reverse asymmetry
    CPG_network_pram.hip_amp_left = CPG_network_pram.hip_amp * (1.0f + 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.hip_amp_right = CPG_network_pram.hip_amp * (1.0f - 0.5f *HIP_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_left = CPG_network_pram.knee_amp * (1.0f + KNEE_TURN_MOD_FACTOR);
    CPG_network_pram.knee_amp_right = CPG_network_pram.knee_amp * (1.0f - KNEE_TURN_MOD_FACTOR);

    // Recompute max_amp
    CPG_network_pram.max_amp = fmaxf(CPG_network_pram.hip_amp_left, CPG_network_pram.knee_amp_left);
    if (CPG_network_pram.max_amp < 1e-3f) CPG_network_pram.max_amp = 1.0f;

    // Reset oscillators (similar to left, but with swapped amps)
    float hip_omega = CPG_network_pram.base_freq * CPG_network_pram.hip_omega_mult;
    set_oscillator_params(FLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(BLH, hip_omega, CPG_network_pram.hip_amp_left, CPG_network_pram.hip_offset);
    set_oscillator_params(FRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    set_oscillator_params(BRH, hip_omega, CPG_network_pram.hip_amp_right, CPG_network_pram.hip_offset);
    float knee_omega = CPG_network_pram.base_freq * CPG_network_pram.knee_omega_mult;
    set_oscillator_params(FLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(BLK, knee_omega, CPG_network_pram.knee_amp_left, CPG_network_pram.knee_offset);
    set_oscillator_params(FRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);
    set_oscillator_params(BRK, knee_omega, CPG_network_pram.knee_amp_right, CPG_network_pram.knee_offset);

    
}





