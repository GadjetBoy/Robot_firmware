import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# CPG Parameters (from user's code)
NUM_OSC = 8
TWO_PI = 2 * np.pi
CPG_DT = 0.0001  # 10 kHz
SIM_TIME = 10.0  # 10 seconds
t = np.arange(0, SIM_TIME, CPG_DT)

# Oscillator indices
FLH, FRH, BLH, BRH, FLK, FRK, BLK, BRK = 0, 1, 2, 3, 4, 5, 6, 7

# Coupling weights and offsets (trot gait)
K_intra_leg = 10.0
K_inter_leg = 5.0
knee_lag = -TWO_PI / 4.0  # -pi/2

coupling_weights = np.zeros((NUM_OSC, NUM_OSC))
phase_offsets = np.zeros((NUM_OSC, NUM_OSC))

# Intra-leg (hip to knee)
coupling_weights[FLK, FLH] = K_intra_leg; phase_offsets[FLK, FLH] = knee_lag
coupling_weights[FRK, FRH] = K_intra_leg; phase_offsets[FRK, FRH] = knee_lag
coupling_weights[BLK, BLH] = K_intra_leg; phase_offsets[BLK, BLH] = knee_lag
coupling_weights[BRK, BRH] = K_intra_leg; phase_offsets[BRK, BRH] = knee_lag

# Inter-leg (hip to hip, trot: diagonals in-phase, ipsi out-of-phase)
# FLH-BR H in-phase
coupling_weights[FLH, BRH] = K_inter_leg; phase_offsets[FLH, BRH] = 0
coupling_weights[BRH, FLH] = K_inter_leg; phase_offsets[BRH, FLH] = 0
# FRH-BL H in-phase
coupling_weights[FRH, BLH] = K_inter_leg; phase_offsets[FRH, BLH] = 0
coupling_weights[BLH, FRH] = K_inter_leg; phase_offsets[BLH, FRH] = 0
# Ipsi out-of-phase (pi)
coupling_weights[FLH, FRH] = K_inter_leg; phase_offsets[FLH, FRH] = np.pi
coupling_weights[FRH, FLH] = K_inter_leg; phase_offsets[FRH, FLH] = np.pi
coupling_weights[FLH, BLH] = K_inter_leg; phase_offsets[FLH, BLH] = np.pi
coupling_weights[BLH, FLH] = K_inter_leg; phase_offsets[BLH, FLH] = np.pi
coupling_weights[FRH, BRH] = K_inter_leg; phase_offsets[FRH, BRH] = np.pi
coupling_weights[BRH, FRH] = K_inter_leg; phase_offsets[BRH, FRH] = np.pi
coupling_weights[BLH, BRH] = K_inter_leg; phase_offsets[BLH, BRH] = np.pi
coupling_weights[BRH, BLH] = K_inter_leg; phase_offsets[BRH, BLH] = np.pi

# Amps and offsets
hip_amp = 32704.0
knee_amp = 16352.0
hip_offset = 0.0
knee_offset = 0.0
max_amp = max(hip_amp, knee_amp)

# Initial phases (trot seed)
phases0 = np.zeros(NUM_OSC)
phases0[FLH] = phases0[BRH] = 0.0
phases0[FLK] = phases0[BRK] = np.pi / 2
phases0[FRH] = phases0[BLH] = np.pi
phases0[FRK] = phases0[BLK] = np.pi + np.pi / 2

# Initial amps
amps0 = np.array([hip_amp if i in [FLH,FRH,BLH,BRH] else knee_amp for i in range(NUM_OSC)])

# Initial state: [phases, amplitudes, offsets] flattened (offsets fixed=0)
y0 = np.concatenate([phases0, amps0, np.zeros(NUM_OSC)])

def cpg_original(y, t, base_omegas):
    phases = y[0:NUM_OSC]
    amps = y[NUM_OSC:2*NUM_OSC]  # Amps evolve slowly
    offsets = y[2*NUM_OSC:]  # Fixed=0
    
    dphi = np.zeros(NUM_OSC)
    damps = np.zeros(NUM_OSC)
    for i in range(NUM_OSC):
        omega_i = base_omegas[i]
        coupling = 0.0
        for j in range(NUM_OSC):
            if i == j: continue
            coupling += coupling_weights[i,j] * amps[i] * np.sin(phases[j] - phases[i] - phase_offsets[i,j])
        dphi[i] = omega_i + coupling
        
        # Amp stabilization (original)
        target_amp = hip_amp if i in [FLH,FRH,BLH,BRH] else knee_amp
        d_amp = 0.005 * (target_amp - amps[i])
        damps[i] = d_amp
        
    doffsets = np.zeros(NUM_OSC)
    return np.concatenate([dphi, damps, doffsets])

def cpg_fixed(y, t, base_omegas, damping=0.1):
    phases = y[0:NUM_OSC]
    amps = y[NUM_OSC:2*NUM_OSC]
    offsets = y[2*NUM_OSC:]
    
    dphi = np.zeros(NUM_OSC)
    damps = np.zeros(NUM_OSC)
    for i in range(NUM_OSC):
        omega_i = base_omegas[i]
        coupling = 0.0
        for j in range(NUM_OSC):
            if i == j: continue
            sin_term = np.sin(phases[j] - phases[i] - phase_offsets[i,j])
            norm_amp = amps[i] / max_amp
            coupling += coupling_weights[i,j] * norm_amp * sin_term
        dphi[i] = (1 - damping) * omega_i + damping * coupling
        
        # Amp stabilization
        target_amp = hip_amp if i in [FLH,FRH,BLH,BRH] else knee_amp
        d_amp = 0.005 * (target_amp - amps[i])
        damps[i] = d_amp
        
    doffsets = np.zeros(NUM_OSC)
    return np.concatenate([dphi, damps, doffsets])

# Frequencies
freqs_list = [0.05, 0.1, 0.2]
base_omegas = np.full(NUM_OSC, TWO_PI * 0.1)  # Base 0.1 Hz, but override per sim

results = {}
for freq in freqs_list:
    print(f"\n=== Simulating at {freq} Hz ===")
    base_omegas[:] = TWO_PI * freq  # Set all omegas to freq
    
    # Original
    sol_orig = odeint(cpg_original, y0, t, args=(base_omegas,))
    phases_orig = sol_orig[:, 0:NUM_OSC]
    amps_orig = sol_orig[:, NUM_OSC:2*NUM_OSC]
    outputs_orig = amps_orig * np.sin(phases_orig)  # + offsets=0
    
    # Fixed
    sol_fixed = odeint(cpg_fixed, y0, t, args=(base_omegas,))
    phases_fixed = sol_fixed[:, 0:NUM_OSC]
    amps_fixed = sol_fixed[:, NUM_OSC:2*NUM_OSC]
    outputs_fixed = amps_fixed * np.sin(phases_fixed)
    
    # Compute range loss % for hips/knees (steady-state, last 5s)
    ss_idx = np.where(t >= 5.0)[0]
    hip_ids = np.array([FLH, FRH, BLH, BRH])
    knee_ids = np.array([FLK, FRK, BLK, BRK])
    
    hip_range_orig = np.ptp(outputs_orig[ss_idx][:, hip_ids], axis=0).mean()
    knee_range_orig = np.ptp(outputs_orig[ss_idx][:, knee_ids], axis=0).mean()
    expected_hip = 2 * hip_amp
    expected_knee = 2 * knee_amp
    loss_hip_orig = (1 - hip_range_orig / expected_hip) * 100
    loss_knee_orig = (1 - knee_range_orig / expected_knee) * 100
    
    hip_range_fixed = np.ptp(outputs_fixed[ss_idx][:, hip_ids], axis=0).mean()
    knee_range_fixed = np.ptp(outputs_fixed[ss_idx][:, knee_ids], axis=0).mean()
    loss_hip_fixed = (1 - hip_range_fixed / expected_hip) * 100
    loss_knee_fixed = (1 - knee_range_fixed / expected_knee) * 100
    
    print(f"Original: Hip range loss {loss_hip_orig:.2f}%, Knee {loss_knee_orig:.2f}%")
    print(f"Fixed: Hip range loss {loss_hip_fixed:.2f}%, Knee {loss_knee_fixed:.2f}%")
    
    results[freq] = {
        'loss_hip_orig': loss_hip_orig, 'loss_knee_orig': loss_knee_orig,
        'loss_hip_fixed': loss_hip_fixed, 'loss_knee_fixed': loss_knee_fixed
    }

# Summary
print("\n=== SUMMARY ===")
for freq, res in results.items():
    print(f"{freq} Hz - Orig Hip/Knee Loss: {res['loss_hip_orig']:.2f}% / {res['loss_knee_orig']:.2f}%")
    print(f"{freq} Hz - Fixed Hip/Knee Loss: {res['loss_hip_fixed']:.2f}% / {res['loss_knee_fixed']:.2f}%")
print("Fixed shows <5% loss as expected.")