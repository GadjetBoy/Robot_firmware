import numpy as np
from scipy.optimize import differential_evolution
import matplotlib.pyplot as plt

iter_count = 0  # Global counter for progress

def callback(xk, convergence):
    global iter_count
    iter_count += 1
    print(f"Progress: {iter_count / 2000 * 100:.2f}% complete (Iteration {iter_count})")
    print("Current best error:", compute_constraints(xk))

# Optimized: Angles as vars
def compute_constraints(vars_all):
    dims_real = vars_all[:12]
    angles = vars_all[12:]
   
    C = dims_real[0] + 1j * dims_real[1]
    D = dims_real[2] + 1j * dims_real[3]
    F = dims_real[4] + 1j * dims_real[5]
    G = dims_real[6] + 1j * dims_real[7]
    H = dims_real[8] + 1j * dims_real[9]
    P0 = dims_real[10] + 1j * dims_real[11]
   
    A = 0 + 0j
    B = 1 + 0j
   
    # M=11 from Salto Eq. 1 (flipped y for down)
    Pj = np.array([0 + 0j, 0 - 0.8j, 0 - 1.125j, 0 - 1.5j, 0 - 1.875j, 0 - 2.25j, 0 - 2.625j, 0 - 3j, 0 - 3.375j, 0 - 3.75j, 0 - 4.125j])
    M = len(Pj)
   
    wj = np.linspace(0, 3.79, M) # 217 deg rotation
   
    error = 0.0
    for j in range(1, M):
        idx = 4*(j-1)
        phi = angles[idx]
        q = angles[idx+1]
        h = angles[idx+2]
        l = angles[idx+3]
       
        Sj = np.exp(1j * wj[j])
        Qj = np.exp(1j * phi)
        Rj = np.exp(1j * q)
        Tj = np.exp(1j * h)
        Uj = np.exp(1j * l)
       
        L1 = A + Qj*(C - A) + Rj*(P0 - C) - Pj[j]
        L2 = B + Sj*(D - B) + Tj*(G - D) + Rj*(P0 - G) - Pj[j]
        L3 = B + Sj*(F - B) + Uj*(H - F) + Rj*(P0 - H) - Pj[j]
       
        error += np.abs(L1)**2 + np.abs(L2)**2 + np.abs(L3)**2
        error += 200 * (np.real(L1))**2 # Strong penalty for x=0
   
    # Continuous straightness penalty
    theta_sample = np.linspace(0, 3.79, 30)
    for th in theta_sample:
        j_frac = (th / 3.79) * (M - 1)
        idx = min(int(j_frac) * 4, len(angles) - 4)
        frac = j_frac - int(j_frac)
        phi = angles[idx] * (1 - frac) + angles[idx + 4] * frac if idx + 4 < len(angles) else angles[idx]
        q = angles[idx + 1] * (1 - frac) + angles[idx + 5] * frac if idx + 5 < len(angles) else angles[idx + 1]
        h = angles[idx + 2] * (1 - frac) + angles[idx + 6] * frac if idx + 6 < len(angles) else angles[idx + 2]
        l = angles[idx + 3] * (1 - frac) + angles[idx + 7] * frac if idx + 7 < len(angles) else angles[idx + 3]
       
        Sj = np.exp(1j * th)
        Qj = np.exp(1j * phi)
        Rj = np.exp(1j * q)
        Tj = np.exp(1j * h)
        Uj = np.exp(1j * l)
       
        P = A + Qj*(C - A) + Rj*(P0 - C)
        error += 200 * (np.real(P))**2 # Penalize x !=0
   
    return error
# Bounds for differential_evolution
bounds = [(-3,3)]*12 + [(-np.pi, np.pi)]*40
# Run global opt
res = differential_evolution(compute_constraints, bounds, tol=1e-8, maxiter=2000, popsize=20, workers=1, callback=callback)
print("Success:", res.success)
print("Final error:", res.fun)
print("Message:", res.message)
solved_real = res.x[:12]
angles_opt = res.x[12:]
C = solved_real[0] + 1j * solved_real[1]
D = solved_real[2] + 1j * solved_real[3]
F = solved_real[4] + 1j * solved_real[5]
G = solved_real[6] + 1j * solved_real[7]
H = solved_real[8] + 1j * solved_real[9]
P0 = solved_real[10] + 1j * solved_real[11]
print(f"Solved pivots: C={C}, D={D}, F={F}, G={G}, H={H}, P0={P0}")
A = 0 + 0j
B = 1 + 0j
lengths = {
    'Input link AC': np.abs(C - A),
    'Coupler P0C': np.abs(P0 - C),
    'Ternary BD': np.abs(D - B),
    'Floating GD': np.abs(G - D),
    'Coupler P0G': np.abs(P0 - G),
    'Ternary BF': np.abs(F - B),
    'Floating HF': np.abs(H - F),
    'Coupler P0H': np.abs(P0 - H),
    'Coupler CG': np.abs(G - C),
    'Coupler GH': np.abs(H - G)
}
print("Link lengths (normalized):", lengths)
# Sim - always run
Pj = np.array([0 + 0j, 0 - 0.8j, 0 - 1.125j, 0 - 1.5j, 0 - 1.875j, 0 - 2.25j, 0 - 2.625j, 0 - 3j, 0 - 3.375j, 0 - 3.75j, 0 - 4.125j])
M = len(Pj)
wj = np.linspace(0, 3.79, M)
theta = np.linspace(0, 3.79, 100)
path_x = []
path_y = []
for th in theta:
    j_frac = (th / 3.79) * (M - 1)
    idx = int(j_frac) * 4
    frac = j_frac - int(j_frac)
    if idx < len(angles_opt) - 4:
        phi = angles_opt[idx] * (1 - frac) + angles_opt[idx + 4] * frac
        q = angles_opt[idx + 1] * (1 - frac) + angles_opt[idx + 5] * frac
        h = angles_opt[idx + 2] * (1 - frac) + angles_opt[idx + 6] * frac
        l = angles_opt[idx + 3] * (1 - frac) + angles_opt[idx + 7] * frac
    else:
        phi, q, h, l = angles_opt[-4:]
       
    Sj = np.exp(1j * th)
    Qj = np.exp(1j * phi)
    Rj = np.exp(1j * q)
    Tj = np.exp(1j * h)
    Uj = np.exp(1j * l)
       
    P = A + Qj*(C - A) + Rj*(P0 - C)
    path_x.append(np.real(P))
    path_y.append(np.imag(P))
# Print max x dev
print("Max X deviation in path:", np.max(np.abs(path_x)))
   
plt.figure()
plt.plot(path_x, path_y, 'b-', label='Foot Path')
plt.scatter([0]*len(Pj), [np.imag(p) for p in Pj], color='r', label='Precision Points')
plt.title('Simulated Foot Path (Check Verticality)')
plt.xlabel('X (should be ~0)')
plt.ylabel('Y (stroke down)')
plt.legend()
plt.grid(True)
plt.show()