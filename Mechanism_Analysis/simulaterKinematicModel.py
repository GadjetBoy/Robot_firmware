"""
Forward Kinematics Simulator - Continuum Leg
Model: TPU flexible leg with d 7.0 v.1.1
Visualizes a tubular leg body along the backbone curve.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import art3d

# Parameters - TPU flexible leg with d 7.0 v.1.1
d = 7.0   # Tendon radial distance (mm) - TPU leg v.1.1
leg_radius = 6.0   # Outer radius of leg body (mm) - match your STL cross-section
STL_PATH =  r"C:\Users\SAHAN\OneDrive\Documents\solidworks\design 1.1\TPU flexible leg with d 7.0 v.1.1" # set to load STL for radius hint
if STL_PATH:
    try:
        from stl import mesh as stl_mesh
        _stl = stl_mesh.Mesh.from_file(STL_PATH)
        _pts = _stl.vectors.reshape(-1, 3)
        _r = np.sqrt(_pts[:, 0]**2 + _pts[:, 1]**2)
        leg_radius = float(np.percentile(_r, 95))  # ~outer radius from STL
        print(f"STL loaded: leg_radius set to {leg_radius:.1f} mm")
    except Exception:
        pass  # Use default leg_radius

l_min = 40.0  # Min length for sliders (mm)
l_max = 86.0  # Max length for sliders (mm)
l_init = 86.0  # Initial length (mm)
axis_length = 15.0  # Length of end-effector axes (mm)
tube_resolution = 12  # Number of segments around tube circumference

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
ax.set_title('Continuum Leg Simulator - TPU d7.0 v.1.1')

# Sliders for l1, l2, l3, l4
ax_l1 = plt.axes([0.1, 0.25, 0.8, 0.03])
ax_l2 = plt.axes([0.1, 0.20, 0.8, 0.03])
ax_l3 = plt.axes([0.1, 0.15, 0.8, 0.03])
ax_l4 = plt.axes([0.1, 0.10, 0.8, 0.03])

sl1 = Slider(ax_l1, 'l1', l_min, l_max, valinit=l_init)
sl2 = Slider(ax_l2, 'l2', l_min, l_max, valinit=l_init)
sl3 = Slider(ax_l3, 'l3', l_min, l_max, valinit=l_init)
sl4 = Slider(ax_l4, 'l4', l_min, l_max, valinit=l_init)


def make_tube(ax, x, y, z, radius, n_theta=12):
    """Create a 3D tubular surface along the backbone curve."""
    n = len(x)
    if n < 2:
        return
    # Tangent vectors
    tx = np.diff(x)
    ty = np.diff(y)
    tz = np.diff(z)
    # Pad to same length
    tangents = np.column_stack([
        np.concatenate([[tx[0]], tx, [tx[-1]]]),
        np.concatenate([[ty[0]], ty, [ty[-1]]]),
        np.concatenate([[tz[0]], tz, [tz[-1]]])
    ])
    tangents = tangents / (np.linalg.norm(tangents, axis=1, keepdims=True) + 1e-10)
    # Reference "up" for first normal
    ref = np.array([0, 0, 1])
    normals = np.zeros_like(tangents)
    normals[0] = ref - np.dot(ref, tangents[0]) * tangents[0]
    n0_norm = np.linalg.norm(normals[0])
    if n0_norm > 1e-8:
        normals[0] /= n0_norm
    else:
        normals[0] = np.array([1, 0, 0])
    # Propagate frame along curve (parallel transport)
    for i in range(1, n):
        axis = np.cross(tangents[i - 1], tangents[i])
        axis_norm = np.linalg.norm(axis)
        if axis_norm > 1e-8:
            axis /= axis_norm
            angle = np.arccos(np.clip(np.dot(tangents[i - 1], tangents[i]), -1, 1))
            r = np.cos(angle)
            normals[i] = normals[i - 1] * r + np.cross(axis, normals[i - 1]) * np.sin(angle) + axis * np.dot(axis, normals[i - 1]) * (1 - r)
        else:
            normals[i] = normals[i - 1]
        normals[i] -= np.dot(normals[i], tangents[i]) * tangents[i]
        n_norm = np.linalg.norm(normals[i])
        if n_norm > 1e-8:
            normals[i] /= n_norm
    binormals = np.cross(tangents, normals)
    # Circle in normal-binormal plane
    theta = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    verts = []
    for i in range(n):
        circle = radius * (np.outer(np.cos(theta), normals[i]) + np.outer(np.sin(theta), binormals[i]))
        for j in range(n_theta):
            verts.append([x[i] + circle[j, 0], y[i] + circle[j, 1], z[i] + circle[j, 2]])
    verts = np.array(verts)
    # Build faces for Poly3DCollection
    faces = []
    for i in range(n - 1):
        for j in range(n_theta):
            j2 = (j + 1) % n_theta
            idx = i * n_theta
            faces.append([idx + j, idx + j2, idx + n_theta + j2, idx + n_theta + j])
    poly = art3d.Poly3DCollection(
        [verts[f] for f in faces],
        facecolors=[0.6, 0.75, 0.9, 0.95],
        edgecolors=None,
        linewidths=0.3,
        alpha=0.95
    )
    ax.add_collection3d(poly)


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
    
    # Parameterize the backbone curve (more points for smooth tube)
    num_points = 31
    sigma_arr = np.linspace(0, s, num_points)
    
    if kappa < 1e-6:
        x = np.zeros(num_points)
        y = np.zeros(num_points)
        z = sigma_arr
    else:
        x = np.cos(phi) * (1 - np.cos(kappa * sigma_arr)) / kappa
        y = np.sin(phi) * (1 - np.cos(kappa * sigma_arr)) / kappa
        z = np.sin(kappa * sigma_arr) / kappa
    
    # Clear and replot - tubular leg body + centerline
    ax.clear()
    make_tube(ax, x, y, z, leg_radius, n_theta=tube_resolution)
    ax.plot(x, y, z, 'k-', lw=1, alpha=0.5)  # Centerline (backbone)
    
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
