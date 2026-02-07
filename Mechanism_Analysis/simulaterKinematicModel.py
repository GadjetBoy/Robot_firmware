import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Parameters
d = 5.50  # Tendon radial distance (mm) - adjust this to your actual value
l_min = 40.0  # Min length for sliders (mm)
l_max = 86.0  # Max length for sliders (mm)
l_init = 86.0  # Initial length (mm)
axis_length = 15.0  # Length of end-effector axes (mm)

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.35)  # Space for sliders

# Initial plot (empty)
line, = ax.plot([], [], [], lw=2)

# Set axis limits
ax.set_xlim(-50, 50)
ax.set_ylim(-50, 50)
ax.set_zlim(0, 100)
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Continuum Leg Simulator')

# Sliders for l1, l2, l3, l4
ax_l1 = plt.axes([0.1, 0.25, 0.8, 0.03])
ax_l2 = plt.axes([0.1, 0.20, 0.8, 0.03])
ax_l3 = plt.axes([0.1, 0.15, 0.8, 0.03])
ax_l4 = plt.axes([0.1, 0.10, 0.8, 0.03])

sl1 = Slider(ax_l1, 'l1', l_min, l_max, valinit=l_init)
sl2 = Slider(ax_l2, 'l2', l_min, l_max, valinit=l_init)
sl3 = Slider(ax_l3, 'l3', l_min, l_max, valinit=l_init)
sl4 = Slider(ax_l4, 'l4', l_min, l_max, valinit=l_init)

# Update function
def update(val):
    l1 = sl1.val
    l2 = sl2.val
    l3 = sl3.val
    l4 = sl4.val
    
    # Compute s, phi, theta, kappa
    s = (l1 + l2 + l3 + l4) / 4
    dl_x = l3 - l1
    dl_y = l4 - l2
    theta = np.sqrt(dl_x**2 + dl_y**2) / (2 * d)
    phi = np.arctan2(dl_y, dl_x) if (np.abs(dl_x) > 1e-6 or np.abs(dl_y) > 1e-6) else 0.0
    kappa = theta / s if s > 0 else 0.0
    
    # Parameterize the backbone curve
    num_points = 11
    sigma_arr = np.linspace(0, s, num_points)
    
    if kappa < 1e-6:
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        z = sigma_arr
    else:
        x = np.cos(phi) * (1 - np.cos(kappa * sigma_arr)) / kappa
        y = np.sin(phi) * (1 - np.cos(kappa * sigma_arr)) / kappa
        z = np.sin(kappa * sigma_arr) / kappa
    
    # Clear and replot backbone
    ax.clear()
    ax.plot(x, y, z, lw=2)
    
    # Re-set limits and labels
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_zlim(0, 100)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    
    # Compute end position
    pos_x, pos_y, pos_z = x[-1], y[-1], z[-1]
    
    # Compute rotation matrix R (standard for constant curvature)
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    R = np.array([
        [cth * cphi**2 + sphi**2, (cth - 1) * sphi * cphi, sth * cphi],
        [(cth - 1) * sphi * cphi, cth * sphi**2 + cphi**2, sth * sphi],
        [-sth * cphi, -sth * sphi, cth]
    ])
    
    # Plot end-effector axes
    ax.quiver(pos_x, pos_y, pos_z, R[0,0], R[1,0], R[2,0], length=axis_length, color='r')  # Tip x-axis
    ax.quiver(pos_x, pos_y, pos_z, R[0,1], R[1,1], R[2,1], length=axis_length, color='g')  # Tip y-axis
    ax.quiver(pos_x, pos_y, pos_z, R[0,2], R[1,2], R[2,2], length=axis_length, color='b')  # Tip z-axis
    
    fig.canvas.draw_idle()

# Attach update to sliders
sl1.on_changed(update)
sl2.on_changed(update)
sl3.on_changed(update)
sl4.on_changed(update)

# Initial update
update(None)

plt.show()