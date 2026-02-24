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
L_MIN, L_MAX = 40.0, 86.0
L_MID = (L_MIN + L_MAX) / 2
S_AMP = (L_MAX - L_MIN) / 2  # ~23 mm
THETA_AMP = 0.4  # Max bend angle (rad) - tune for visible motion


def normalize_cpg(val, min_out=-20000, max_out=20000):
    """Map CPG output (encoder counts) to [-1, 1]."""
    return np.clip((val - min_out) / (max_out - min_out) * 2 - 1, -1, 1)


def hip_knee_to_tendons(hip_out, knee_out, d=D_TENDON):
    """
    Map hip and knee outputs to tendon lengths l1,l2,l3,l4.
    Hip: controls bend direction (phi) and amount (theta) - forward/back swing
    Knee: controls arc length (s) - extend/flex
    """
    # Normalize: hip ~[-16k,16k], knee ~[-23k,0] or [0,32k] depending on offset
    hip_norm = normalize_cpg(hip_out, -18000, 18000)
    knee_norm = normalize_cpg(knee_out, -25000, 32000)
    
    # s: extended (knee+) -> long leg, flexed (knee-) -> short
    s = L_MID + S_AMP * knee_norm
    
    # theta: bend amount from hip swing
    theta = THETA_AMP * (1 + hip_norm) / 2  # 0 when back, theta_amp when forward
    
    # phi = 0: bend in +x direction (forward)
    phi = 0.0
    
    # From s, theta, phi compute dl_x, dl_y
    dl_x = 2 * d * theta * np.cos(phi)
    dl_y = 2 * d * theta * np.sin(phi)
    
    l1 = s - dl_x / 2
    l3 = s + dl_x / 2
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
