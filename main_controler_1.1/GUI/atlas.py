# Save this as stephenson6_example.py and run with: python stephenson6_example.py
# Requires: numpy, matplotlib

import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# Utility helpers
# ---------------------------
def vec(l, theta):
    """Return 2D vector for length l and angle theta (radians)."""
    return np.array([l * np.cos(theta), l * np.sin(theta)])

def normalize_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# ---------------------------
# Mechanism geometry (example values)
# Choose a set known to give interesting coupler shapes (tweak as needed)
# ---------------------------
# Link lengths (units arbitrary)
l1 = 1.0
l2 = 0.6   # input crank (we will sweep theta2)
l3 = 1.2
l4 = 1.1   # coupler between loops (shared)
l5 = 0.9
l6 = 0.7
l7 = 2.0   # ground link between F and A (placed on x-axis)

# Ground pivot coordinates
A = np.array([0.0, 0.0])
F = np.array([l7, 0.0])  # so vector A->F has length l7

# We'll treat angles theta1..theta6 as follows:
# theta2 is input (swept). Unknowns we will solve for: theta1, theta3, theta4, theta5
# We assume link 6 connects E->F and its orientation theta6 will be determined from loop2 residuals (or computed)
# For consistency we'll actually use 4 unknowns: theta1, theta3, theta4, theta5
# (theta6 can be deduced if needed from vector eqns)

# Coupler point: on link3 at fractional position alpha (0..1)
alpha = 0.5

# ---------------------------
# Residual function for Newton solve
# Unknown vector y = [theta1, theta3, theta4, theta5]
# Input: theta2 (given)
# Two vector loop equations yield 4 scalar residuals (loop1 x,y and loop2 x,y)
# Loop1: r1 + r2 + r3 - r4 = 0   (A->B->C->D->A)
# Loop2: r4 + r5 + r6 + r7 = 0   (A->D->E->F->A) where r7 = F->A = (-l7,0)
# Note: angles are absolute (w.r.t x-axis)
# ---------------------------
def residuals(y, theta2):
    th1, th3, th4, th5 = y
    # Build vectors:
    r1 = vec(l1, th1)      # A->B
    r2 = vec(l2, theta2)   # B->C (input)
    r3 = vec(l3, th3)      # C->D
    r4 = vec(l4, th4)      # D->A  (note orientation as D->A)
    r5 = vec(l5, th5)      # D->E
    # r6 is E->F with angle th6 unknown; we will express it symbolically as vector of magnitude l6 and angle th6
    # r7 is F->A (constant): vector from F to A = A - F = (-l7, 0)
    r7 = np.array([-l7, 0.0])
    # Loop1 residual: r1 + r2 + r3 - r4 = 0
    R1 = r1 + r2 + r3 - r4
    # Loop2 residual: r4 + r5 + r6 + r7 = 0 -> unknown r6 must close this loop
    # Re-arrange to find r6 = - (r4 + r5 + r7)
    r6_needed = - (r4 + r5 + r7)
    # The r6_needed vector must have magnitude l6. So we enforce this by two scalar residuals:
    # 1) its x-component minus expected x
    # 2) its y-component minus expected y
    # Alternatively, we use the two equations (r4 + r5 + r6 + r7 = 0) directly: that is two scalar residuals.
    R2 = r4 + r5 + r6_needed + r7  # should be near zero, mathematically zero by construction, but we'll use two scalar eqns below
    # But r6_needed + r4 + r5 + r7 is exactly zero by definition above; we actually need to enforce |r6_needed| = l6
    # So let residuals be:
    # - the loop1 x and y
    # - two conditions making magnitude of r6_needed equal to l6 and its angle consistent
    mag_diff = np.linalg.norm(r6_needed) - l6
    # Also, we can enforce orientation consistency by enforcing cross-product sign or angle difference, but magnitude condition plus loop1 x,y and an angle relation serve.
    # To form a 4x1 residual vector we use:
    return np.array([R1[0], R1[1], mag_diff, 0.0])  # last component zero placeholder; we'll make Newton handle with a small trick

# The above residual vector is not fully ideal because we used a magnitude constraint and a placeholder.
# Instead we'll write a direct 4-scalar residual system:
def residuals_full(y, theta2):
    th1, th3, th4, th5 = y
    r1 = vec(l1, th1)
    r2 = vec(l2, theta2)
    r3 = vec(l3, th3)
    r4 = vec(l4, th4)
    r5 = vec(l5, th5)
    r7 = np.array([-l7, 0.0])
    # Loop1 components:
    R1 = r1[0] + r2[0] + r3[0] - r4[0]
    R2 = r1[1] + r2[1] + r3[1] - r4[1]
    # Loop2 components:
    # r6 = - (r4 + r5 + r7)
    r6 = - (r4 + r5 + r7)
    # Enforce that magnitude of r6 equals l6:
    R3 = r6[0]**2 + r6[1]**2 - l6**2
    # Also enforce that angle of r6 is continuous relative to previous step; but for a self-contained system we require one more scalar equation:
    # Use signed x-component of r6 (or dot with a known direction) to choose branch:
    # We'll enforce the oriented x-component minus expected x_guess (we'll use previous theta guess to help), but that requires passing an extra parameter.
    # A simple consistent additional residual is to enforce that the y-component of r6 has a particular sign or value derived from the previous step.
    # For robustness, we'll use r6_y - r6_y_guess where r6_y_guess is provided externally. To avoid that complexity in this script, we'll set R4 = ???.
    # As a pragmatic approach, we set R4 = (th1 + th3 + th4 + th5) - C where C is not known. To keep a solvable system, we will instead treat theta1 as dependent (express in terms of other angles) and solve for [th3, th4, th5, some aux]. But that's messy.
    # Practical approach: convert to solving 4 scalar equations: loop1 x, loop1 y, loop2 x, loop2 y (i.e., r4 + r5 + r6 + r7 = 0) but r6 has magnitude unknown angle variable th6; so include th6 as unknown instead of one of the thetas that we earlier included.
    # Therefore it's much clearer to pick unknowns vector = [th1, th3, th4, th6] and treat th5 from geometry or vice versa.
    # To keep the script simple and robust, we move to solving unknowns [th1, th3, th4, th6] and compute r5 via vector relation; that's what the runnable code below does.

    return np.array([R1, R2, R3, 0.0])  # placeholder; we'll use the alternative more stable solver below.

# ---------------------------
# Simpler, more robust formulation implemented below:
# Unknowns: y = [th1, th3, th4, th6]
# We then form four scalar residuals:
# Loop1 (x,y): r1 + r2 + r3 - r4 = 0 -> 2 residuals
# Loop2 (x,y): r4 + r5 + r6 + r7 = 0 -> 2 residuals
# But r5 is not known; we will express r5 as vector from D to E, but E position is free; however the algebra is symmetric:
# We'll pick unknowns [th1, th3, th4, th6] and treat r5 as l5 oriented so that r5 = - (r4 + r6 + r7) ; then the magnitude of r5 must equal l5 -> magnitude residual
# That gives: residuals = [loop1_x, loop1_y, loop2_x, loop2_y] where loop2_x/y equals r4 + r5 + r6 + r7 computed with current unknowns.
# This is now consistent because r5 is computed from unknowns and must satisfy magnitude l5 (but we only used vector sum equality). The set of four equations can be:
# (r1 + r2 + r3 - r4) x,y and (r4 + r5 + r6 + r7) x,y with r5 = vec(l5, th5) and th5 unknown... so again th5 must be unknown.
# To avoid endless algebraic juggling in this short script, we will adopt the following practical and widely used approach:
# Use a numerical root finder solving for unknowns [th1, th3, th4, th5, th6] (5 unknowns) and supply one additional constraint (e.g., fix th1 or tie to previous solution). But using 5 unknowns for 4 eqns is underdetermined; the missing equation is the single DOF constraint — we fix the input theta2, so the mechanism is determinate (should have 4 unknowns).
# Conclusion: For a robust, educational script here, I'll implement a solver that treats unknowns [th1, th3, th4, th5] and enforces loop1(x,y) and loop2(x,y) explicitly by allowing r6 to be computed directly from r4,r5,r7 and then enforcing |r6|=l6 as one equation and sign-of-angle condition as the last eqn using continuity with previous step.
# The sign-of-angle condition will be implemented by driving the angle of r6 toward the previous step's r6 angle (this is a continuation method).
# This continuation method requires keeping r6_angle_prev; initial guess chosen near configuration.
# The runnable code below implements that.
# ---------------------------

# ---------------------------
# Main solver + sweep
# ---------------------------
def solve_for_configuration(theta2_vals, initial_guess=None):
    # We'll keep previous r6 angle to provide the 4th residual (continuation constraint)
    pts = []
    r6_angle_prev = -0.5  # initial guess
    # initial guess for unknowns [th1, th3, th4, th5]
    if initial_guess is None:
        y_prev = np.array([0.0, 0.0, 0.0, 0.0])
    else:
        y_prev = initial_guess

    for theta2 in theta2_vals:
        y = y_prev.copy()
        # Newton iterations
        for it in range(60):
            th1, th3, th4, th5 = y
            # vectors
            r1 = vec(l1, th1)
            r2 = vec(l2, theta2)
            r3 = vec(l3, th3)
            r4 = vec(l4, th4)
            r5 = vec(l5, th5)
            r7 = np.array([-l7, 0.0])
            # Loop1 residuals
            R1 = r1[0] + r2[0] + r3[0] - r4[0]
            R2 = r1[1] + r2[1] + r3[1] - r4[1]
            # Loop2: r6 = - (r4 + r5 + r7)
            r6 = - (r4 + r5 + r7)
            mag_r6 = np.linalg.norm(r6)
            # Residual 3: magnitude difference
            R3 = mag_r6 - l6
            # Residual 4: continuation constraint on r6 angle
            r6_angle = np.arctan2(r6[1], r6[0])
            R4 = normalize_angle(r6_angle - r6_angle_prev)  # drive toward previous angle
            R = np.array([R1, R2, R3, R4])
            err = np.linalg.norm(R)
            if err < 1e-9:
                break
            # Numerical Jacobian (finite differences)
            J = np.zeros((4,4))
            eps = 1e-6
            for k in range(4):
                dy = np.zeros(4)
                dy[k] = eps
                y_plus = y + dy
                th1p, th3p, th4p, th5p = y_plus
                r1p = vec(l1, th1p)
                r2p = vec(l2, theta2)
                r3p = vec(l3, th3p)
                r4p = vec(l4, th4p)
                r5p = vec(l5, th5p)
                r6p = - (r4p + r5p + r7)
                R1p = r1p[0] + r2p[0] + r3p[0] - r4p[0]
                R2p = r1p[1] + r2p[1] + r3p[1] - r4p[1]
                R3p = np.linalg.norm(r6p) - l6
                R4p = normalize_angle(np.arctan2(r6p[1], r6p[0]) - r6_angle_prev)
                Rp = np.array([R1p, R2p, R3p, R4p])
                J[:,k] = (Rp - R) / eps
            # Solve linear system for update
            try:
                dy = np.linalg.solve(J, R)
            except np.linalg.LinAlgError:
                # singular Jacobian -> small random perturbation and continue
                y += 1e-4 * (np.random.rand(4)-0.5)
                continue
            y -= dy
        # after convergence, compute coupler point (on link3)
        th1, th3, th4, th5 = y
        r1 = vec(l1, th1)
        r2 = vec(l2, theta2)
        # position B = A + r1
        B = A + r1
        C = B + r2
        # coupler point on link C->D (r3) at fraction alpha from C toward D:
        r3 = vec(l3, th3)
        P = C + alpha * r3
        pts.append(P)
        # update continuation
        r4 = vec(l4, th4)
        r5 = vec(l5, th5)
        r6 = - (r4 + r5 + np.array([-l7,0.0]))
        r6_angle_prev = np.arctan2(r6[1], r6[0])
        y_prev = y
    return np.array(pts)

# Sweep input angle:
theta2_vals = np.linspace(-0.9, 0.9, 120)  # radians; adjust as appropriate for your mechanism's valid range
points = solve_for_configuration(theta2_vals)

# Fit straight line y = a x + b via least squares
xs = points[:,0]
ys = points[:,1]
A_fit = np.vstack([xs, np.ones_like(xs)]).T
a, b = np.linalg.lstsq(A_fit, ys, rcond=None)[0]
# Compute orthogonal distances to line
# distance from (x0,y0) to line ax - y + b = 0 => |a x0 - y0 + b| / sqrt(a^2 + 1)
distances = np.abs(a*xs - ys + b) / np.sqrt(a*a + 1)
rms = np.sqrt(np.mean(distances**2))

print("Fitted line: y = {:.6f} x + {:.6f}".format(a,b))
print("RMS orthogonal error (straightness): {:.6e}".format(rms))

# Plot
plt.figure(figsize=(7,5))
plt.plot(xs, ys, marker='o', linestyle='None', label='coupler path')
x_line = np.linspace(xs.min()-0.2, xs.max()+0.2, 100)
y_line = a * x_line + b
plt.plot(x_line, y_line, linestyle='-', label='fitted line')
plt.title("Stephenson-type coupler path and fitted straight line\nRMS error = {:.3e}".format(rms))
plt.xlabel("x")
plt.ylabel("y")
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
