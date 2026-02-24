"""
CPG-to-Continuum Leg Mapping
Python port of CPG dynamics from cpg.c / gate.c for a single leg.
Maps hip+knee oscillator outputs to tendon lengths (l1,l2,l3,l4) for continuum leg FK.
"""
import numpy as np

# Constants from cpg.h / gate.c
TWO_PI = 2.0 * np.pi
CPG_FREQ = 0.09      # Hz (Trot, Crawl)
CPG_CREEP_FREQ = 0.08  # Hz (Creep)
CPG_DT = 1.0 / 500.0  # 500 Hz update rate
KH_OFFSET = 4.0

# Continuum leg parameters
D_TENDON = 7.0   # mm, tendon radial distance
L_MIN, L_MAX = 40.0, 86.0 * 2
L_MID = (L_MIN + L_MAX) / 2
S_AMP = (L_MAX - L_MIN) / 2  # ~23 mm
THETA_AMP = 0.4  # Max bend angle (rad) - tune for visible motion


def normalize_cpg(val, min_out=-20000, max_out=20000):
    """Map CPG output (encoder counts) to [-1, 1]."""
    return np.clip((val - min_out) / (max_out - min_out) * 2 - 1, -1, 1)


def hip_knee_to_tendons(hip_out, knee_out, d=D_TENDON):
    """
    Map FRH (hip) and FRK (knee) CPG outputs to tendon lengths l1,l2,l3,l4.
    
    CRITICAL: l1 and l3 NEVER move together - one compresses, other extends (same amount).
    CRITICAL: l2 and l4 NEVER move together - one compresses, other extends (same amount).
    
    CPG outputs are NEVER both positive and negative at once - at each moment
    hip is either + or -, knee is either + or -.
    
    FRH positive: l1 compresses, l3 extends (same amount)
    FRH negative: l3 compresses, l1 extends (same amount)
    FRK positive: l2 compresses, l4 extends (same amount)
    FRK negative: l4 compresses, l2 extends (same amount)
    """
    # Scale: CPG outputs to differential in mm. |dl/2| must stay within [L_MIN,L_MAX] of s.
    # Knee range ~-53k to +7k is larger than hip ±15k - scale to fit S_AMP.
    HIP_SCALE = S_AMP / 18000.0   # hip ±15k -> dl_x up to ~55
    KNEE_SCALE = S_AMP / 55000.0  # knee ±55k -> dl_y up to ~66 (clamped)
    
    # Hip controls l1,l3 ONLY. dl_x = l3 - l1. Hip + -> dl_x + -> l1 short, l3 long
    dl_x = hip_out * HIP_SCALE
    # Knee controls l2,l4 ONLY. dl_y = l4 - l2. Knee + -> dl_y + -> l2 short, l4 long
    dl_y = knee_out * KNEE_SCALE
    
    # Base arc length (neutral)
    s = L_MID
    
    # l1,l3: ALWAYS opposite. l1 compresses by dl_x/2, l3 extends by dl_x/2 (when dl_x>0)
    l1 = s - dl_x / 2
    l3 = s + dl_x / 2
    # l2,l4: ALWAYS opposite
    l2 = s - dl_y / 2
    l4 = s + dl_y / 2
    
    return np.clip([l1, l2, l3, l4], L_MIN, L_MAX)


class SingleLegCPG:
    """
    Two-oscillator CPG (hip, knee) for one leg.
    Replicates dynamics from cpg_update() and gate.c gaits.
    """
    def __init__(self, gait='trot'):
        self.gait = gait
        self.phase_hip = 0.0
        self.phase_knee = 0.0
        self.duty_cycle = 0.5
        self.hip_amp = 16352.0
        self.knee_amp = 32704.0
        self.hip_offset = 0.0
        self.knee_offset = -23000.0
        self.hip_omega_mult = 1.2
        self.knee_omega_mult = 1.8
        self.base_freq = TWO_PI * CPG_FREQ
        self._set_gait(gait)
    
    def _set_gait(self, gait):
        self.gait = gait
        if gait == 'trot':
            self.duty_cycle = 0.5
            self.base_freq = TWO_PI * CPG_FREQ
            self.hip_amp = 16352.0 * 0.9
            self.knee_amp = 32704.0 * 0.92
            self.knee_offset = -23000.0
            self.hip_omega_mult = 1.2
            self.knee_omega_mult = 1.8
            self.phase_hip = 0.0
            self.phase_knee = TWO_PI / 4.0  # knee lags hip
        elif gait == 'crawl':
            self.duty_cycle = 0.5
            self.base_freq = TWO_PI * CPG_FREQ
            self.hip_amp = 16000.0 * 0.9
            self.knee_amp = 16000.0 * 0.92
            self.knee_offset = 0.0
            self.hip_omega_mult = 1.2
            self.knee_omega_mult = 1.5
            self.phase_hip = 0.0
            self.phase_knee = 0.0
        elif gait == 'creep':
            self.duty_cycle = 0.75
            self.base_freq = TWO_PI * CPG_CREEP_FREQ
            self.hip_amp = 6000.0
            self.knee_amp = 19000.0
            self.knee_offset = -15000.0
            self.hip_omega_mult = 1.0
            self.knee_omega_mult = 1.0
            self.phase_hip = TWO_PI * 0.25
            self.phase_knee = TWO_PI * 0.25 - TWO_PI / KH_OFFSET
        else:
            self.phase_hip = 0.0
            self.phase_knee = TWO_PI / 4.0
    
    def step(self, dt=None):
        if dt is None:
            dt = CPG_DT * 2  # Slower for animation
        duty = max(self.duty_cycle, 0.1)
        # Asymmetric timing (warped frequency) - from cpg_update
        alpha_hip = 1.0 / (2.0 * duty) if self.phase_hip >= np.pi else 1.0 / (2.0 * (1.0 - duty))
        alpha_knee = 1.0 / (2.0 * duty) if self.phase_knee >= np.pi else 1.0 / (2.0 * (1.0 - duty))
        omega_hip = self.base_freq * self.hip_omega_mult * alpha_hip
        omega_knee = self.base_freq * self.knee_omega_mult * alpha_knee
        # Coupling: knee lags hip
        K_intra = 8.0
        knee_lag = -TWO_PI / KH_OFFSET
        coupling = K_intra * np.sin(self.phase_hip - self.phase_knee - knee_lag)
        # Update phases
        self.phase_hip += omega_hip * dt
        self.phase_knee += (omega_knee + coupling * 0.1) * dt
        self.phase_hip = np.fmod(self.phase_hip, TWO_PI)
        self.phase_knee = np.fmod(self.phase_knee, TWO_PI)
        if self.phase_hip < 0:
            self.phase_hip += TWO_PI
        if self.phase_knee < 0:
            self.phase_knee += TWO_PI
        # Outputs
        hip_out = self.hip_offset + self.hip_amp * np.sin(self.phase_hip)
        knee_out = self.knee_offset + self.knee_amp * np.sin(self.phase_knee)
        return hip_out, knee_out
    
    def get_tendon_lengths(self, dt=None):
        """Step CPG and return l1,l2,l3,l4 for continuum leg."""
        hip_out, knee_out = self.step(dt)
        return hip_knee_to_tendons(hip_out, knee_out)
