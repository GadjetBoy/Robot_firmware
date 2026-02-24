"""
Continuum Leg Simulator - Professional Edition
Publication-ready visualization, animation export, CPG integration.
Model: TPU flexible leg with d 7.0 v.1.1
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from mpl_toolkits.mplot3d import art3d
import matplotlib
matplotlib.rcParams['font.family'] = 'serif'
matplotlib.rcParams['font.serif'] = ['Times New Roman', 'DejaVu Serif', 'serif']
matplotlib.rcParams['font.size'] = 11
matplotlib.rcParams['axes.labelsize'] = 12
matplotlib.rcParams['axes.titlesize'] = 14

# Try importing CPG and animation support
try:
    from cpg_continuum import SingleLegCPG, hip_knee_to_tendons
    HAS_CPG = True
except ImportError:
    HAS_CPG = False

# Parameters
D = 7.0
LEG_HALFWIDTH = 6.0
L_MIN, L_MAX, L_INIT = 40.0, 86.0, 86.0
AXIS_LEN = 12.0
N_SEGMENTS, N_PROFILE = 12, 16
WAIST_RATIO, PLATE_RATIO = 0.6, 0.95
END_HW = LEG_HALFWIDTH * 0.7

# Publication settings
DPI = 150
FIG_WIDTH, FIG_HEIGHT = 10, 8
BG_COLOR = '#f8f9fa'
LEG_GREY = (0.42, 0.42, 0.48, 0.98)
PLATE_GREY = (0.52, 0.52, 0.58, 0.98)
EDGE_COLOR = (0.28, 0.28, 0.34, 0.85)


def _rounded_square_points(halfwidth, n_pts, normal, binormal):
    angles = np.linspace(0, 2 * np.pi, n_pts, endpoint=False)
    pts = []
    for a in angles:
        c, s = abs(np.cos(a)), abs(np.sin(a))
        r = halfwidth / max(c, s, 1e-8)
        pts.append(r * np.cos(a) * normal + r * np.sin(a) * binormal)
    return np.array(pts)


def _compute_frames(x, y, z):
    n = len(x)
    tx, ty, tz = np.diff(x), np.diff(y), np.diff(z)
    tangents = np.column_stack([
        np.concatenate([[tx[0]], tx, [tx[-1]]]),
        np.concatenate([[ty[0]], ty, [ty[-1]]]),
        np.concatenate([[tz[0]], tz, [tz[-1]]])
    ])
    tangents = tangents / (np.linalg.norm(tangents, axis=1, keepdims=True) + 1e-10)
    ref = np.array([0, 0, 1])
    normals = np.zeros_like(tangents)
    normals[0] = ref - np.dot(ref, tangents[0]) * tangents[0]
    n0 = np.linalg.norm(normals[0])
    normals[0] = (normals[0] / n0) if n0 > 1e-8 else np.array([1, 0, 0])
    for i in range(1, n):
        axis = np.cross(tangents[i - 1], tangents[i])
        an = np.linalg.norm(axis)
        if an > 1e-8:
            axis /= an
            angle = np.arccos(np.clip(np.dot(tangents[i - 1], tangents[i]), -1, 1))
            c, s = np.cos(angle), np.sin(angle)
            normals[i] = normals[i - 1] * c + np.cross(axis, normals[i - 1]) * s + axis * np.dot(axis, normals[i - 1]) * (1 - c)
        else:
            normals[i] = normals[i - 1]
        normals[i] -= np.dot(normals[i], tangents[i]) * tangents[i]
        nn = np.linalg.norm(normals[i])
        if nn > 1e-8:
            normals[i] /= nn
    binormals = np.cross(tangents, normals)
    return tangents, normals, binormals


def _add_rect_connector(px, py, pz, tangent, normal, binormal, halfw, length, n_pts):
    pts0 = _rounded_square_points(halfw, n_pts, normal, binormal)
    verts, faces = [], []
    for i in range(2):
        off = i * length * tangent
        for j in range(n_pts):
            verts.append((np.array([px, py, pz]) + off + pts0[j]).tolist())
    verts = np.array(verts)
    for j in range(n_pts):
        j2 = (j + 1) % n_pts
        faces.append([verts[j].tolist(), verts[j2].tolist(), verts[n_pts + j2].tolist(), verts[n_pts + j].tolist()])
    return faces


def make_segmented_leg(ax, x, y, z, halfwidth, n_seg, n_pts, waist_ratio, plate_ratio, end_hw):
    n = len(x)
    if n < 2:
        return
    tangents, normals, binormals = _compute_frames(x, y, z)
    all_faces, plate_faces = [], []
    conn_len = 5.0
    fc = _add_rect_connector(x[0], y[0], z[0], -tangents[0], normals[0], binormals[0], end_hw, conn_len, n_pts)
    all_faces.extend(fc)
    seg_idx = np.linspace(0, n - 1, n_seg + 1, dtype=int)
    seg_idx[-1] = n - 1
    for k in range(n_seg):
        i0, i1 = seg_idx[k], seg_idx[k + 1]
        imid = (i0 + i1) // 2
        widths = [halfwidth * plate_ratio, halfwidth, halfwidth * waist_ratio, halfwidth, halfwidth * plate_ratio]
        pts_idx = [i0, i0 + (imid - i0) // 2, imid, i1 - (i1 - imid) // 2, i1]
        verts_seg = []
        for pi, w in zip(pts_idx, widths):
            pts_2d = _rounded_square_points(w, n_pts, normals[pi], binormals[pi])
            for j in range(n_pts):
                verts_seg.append((np.array([x[pi], y[pi], z[pi]]) + pts_2d[j]).tolist())
        verts_seg = np.array(verts_seg)
        for ring in range(4):
            for j in range(n_pts):
                j2 = (j + 1) % n_pts
                o, o2 = ring * n_pts, (ring + 1) * n_pts
                q = [verts_seg[o + j], verts_seg[o + j2], verts_seg[o2 + j2], verts_seg[o2 + j]]
                face = [[float(v) for v in p] for p in q]
                if ring in (0, 3):
                    plate_faces.append(face)
                else:
                    all_faces.append(face)
    fc2 = _add_rect_connector(x[-1], y[-1], z[-1], tangents[-1], normals[-1], binormals[-1], end_hw, conn_len, n_pts)
    all_faces.extend(fc2)
    poly = art3d.Poly3DCollection(all_faces, facecolors=LEG_GREY, edgecolors=EDGE_COLOR, linewidths=0.3, alpha=0.98)
    ax.add_collection3d(poly)
    poly_plate = art3d.Poly3DCollection(plate_faces, facecolors=PLATE_GREY, edgecolors=EDGE_COLOR, linewidths=0.25, alpha=0.98)
    ax.add_collection3d(poly_plate)


def tendon_to_backbone(l1, l2, l3, l4, d, num_pts=61):
    s = (l1 + l2 + l3 + l4) / 4
    dl_x, dl_y = l3 - l1, l4 - l2
    theta = np.sqrt(dl_x**2 + dl_y**2) / (2 * d)
    phi = np.arctan2(dl_y, dl_x) if (np.abs(dl_x) > 1e-6 or np.abs(dl_y) > 1e-6) else 0.0
    kappa = theta / s if s > 0 else 0.0
    sigma = np.linspace(0, s, num_pts)
    if kappa < 1e-6:
        x, y, z = np.zeros(num_pts), np.zeros(num_pts), sigma
    else:
        x = np.cos(phi) * (1 - np.cos(kappa * sigma)) / kappa
        y = np.sin(phi) * (1 - np.cos(kappa * sigma)) / kappa
        z = np.sin(kappa * sigma) / kappa
    return x, y, z


def setup_figure():
    fig = plt.figure(figsize=(FIG_WIDTH, FIG_HEIGHT), dpi=DPI, facecolor=BG_COLOR)
    ax = fig.add_subplot(111, projection='3d', facecolor=BG_COLOR)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.92, bottom=0.28)
    ax.set_xlim(-45, 45)
    ax.set_ylim(-45, 45)
    ax.set_zlim(0, 100)
    ax.set_xlabel('X (mm)', fontsize=11)
    ax.set_ylabel('Y (mm)', fontsize=11)
    ax.set_zlabel('Z (mm)', fontsize=11)
    ax.set_title('Continuum Leg – CPG Gait Simulation', fontsize=14, fontweight='bold')
    ax.tick_params(labelsize=9)
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('0.9')
    ax.yaxis.pane.set_edgecolor('0.9')
    ax.zaxis.pane.set_edgecolor('0.9')
    ax.grid(True, linestyle='--', alpha=0.6)
    return fig, ax


def main():
    fig, ax = setup_figure()
    
    # Sliders
    ax_l1 = fig.add_axes([0.15, 0.18, 0.7, 0.02])
    ax_l2 = fig.add_axes([0.15, 0.14, 0.7, 0.02])
    ax_l3 = fig.add_axes([0.15, 0.10, 0.7, 0.02])
    ax_l4 = fig.add_axes([0.15, 0.06, 0.7, 0.02])
    sl1 = Slider(ax_l1, r'$l_1$', L_MIN, L_MAX, valinit=L_INIT, color='#4a90d9')
    sl2 = Slider(ax_l2, r'$l_2$', L_MIN, L_MAX, valinit=L_INIT, color='#4a90d9')
    sl3 = Slider(ax_l3, r'$l_3$', L_MIN, L_MAX, valinit=L_INIT, color='#4a90d9')
    sl4 = Slider(ax_l4, r'$l_4$', L_MIN, L_MAX, valinit=L_INIT, color='#4a90d9')
    for s in [sl1, sl2, sl3, sl4]:
        s.label.set_fontsize(10)
    
    # Gait selector
    ax_gait = fig.add_axes([0.02, 0.14, 0.08, 0.12])
    radio = RadioButtons(ax_gait, ('Manual', 'Trot', 'Crawl', 'Creep'), active=0)
    for label in radio.labels:
        label.set_fontsize(9)
    
    # Buttons
    ax_ss = fig.add_axes([0.02, 0.06, 0.08, 0.04])
    ax_anim = fig.add_axes([0.02, 0.01, 0.08, 0.04])
    btn_screenshot = Button(ax_ss, 'Screenshot', color='#e8e8e8', hovercolor='#d0d0d0')
    btn_animate = Button(ax_anim, 'Export Animation', color='#e8e8e8', hovercolor='#d0d0d0')
    
    cpg = SingleLegCPG('trot') if HAS_CPG else None
    
    def update(val=None):
        if radio.value_selected == 'Manual' or not HAS_CPG:
            l1, l2, l3, l4 = sl1.val, sl2.val, sl3.val, sl4.val
        else:
            l1, l2, l3, l4 = cpg.get_tendon_lengths(dt=0.02)
            sl1.set_val(l1)
            sl2.set_val(l2)
            sl3.set_val(l3)
            sl4.set_val(l4)
        
        x, y, z = tendon_to_backbone(l1, l2, l3, l4, D)
        ax.clear()
        make_segmented_leg(ax, x, y, z, LEG_HALFWIDTH, N_SEGMENTS, N_PROFILE, WAIST_RATIO, PLATE_RATIO, END_HW)
        ax.set_xlim(-45, 45)
        ax.set_ylim(-45, 45)
        ax.set_zlim(0, 100)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('Continuum Leg – CPG Gait Simulation')
        ax.set_facecolor(BG_COLOR)
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        pos_x, pos_y, pos_z = x[-1], y[-1], z[-1]
        ax.quiver(pos_x, pos_y, pos_z, 1, 0, 0, length=AXIS_LEN, color='#c0392b', linewidth=1.5)
        ax.quiver(pos_x, pos_y, pos_z, 0, 1, 0, length=AXIS_LEN, color='#27ae60', linewidth=1.5)
        ax.quiver(pos_x, pos_y, pos_z, 0, 0, 1, length=AXIS_LEN, color='#2980b9', linewidth=1.5)
        fig.canvas.draw_idle()
    
    def on_gait_select(label):
        if HAS_CPG and label != 'Manual':
            cpg._set_gait(label.lower())
        if anim_running[0] or radio.value_selected != 'Manual':
            update()
    
    def on_screenshot(event):
        fname = 'continuum_leg_screenshot.png'
        fig.savefig(fname, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none', bbox_inches='tight')
        print(f'Saved: {fname}')
    
    def on_animate(event):
        if not HAS_CPG:
            print('CPG module not found. Install cpg_continuum.py')
            return
        gait = radio.value_selected
        if gait == 'Manual':
            gait = 'trot'
        cpg._set_gait(gait.lower())
        n_frames = 120
        frames = []
        for _ in range(n_frames):
            l1, l2, l3, l4 = cpg.get_tendon_lengths(dt=0.04)
            x, y, z = tendon_to_backbone(l1, l2, l3, l4, D)
            frames.append((x, y, z))
        try:
            from matplotlib.animation import FuncAnimation, PillowWriter
            def anim_frame(i):
                ax.clear()
                x, y, z = frames[i]
                make_segmented_leg(ax, x, y, z, LEG_HALFWIDTH, N_SEGMENTS, N_PROFILE, WAIST_RATIO, PLATE_RATIO, END_HW)
                ax.set_xlim(-45, 45)
                ax.set_ylim(-45, 45)
                ax.set_zlim(0, 100)
                ax.set_xlabel('X (mm)')
                ax.set_ylabel('Y (mm)')
                ax.set_zlabel('Z (mm)')
                ax.set_facecolor(BG_COLOR)
                ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
                pos_x, pos_y, pos_z = x[-1], y[-1], z[-1]
                ax.quiver(pos_x, pos_y, pos_z, 1, 0, 0, length=AXIS_LEN, color='#c0392b', linewidth=1.5)
                ax.quiver(pos_x, pos_y, pos_z, 0, 1, 0, length=AXIS_LEN, color='#27ae60', linewidth=1.5)
                ax.quiver(pos_x, pos_y, pos_z, 0, 0, 1, length=AXIS_LEN, color='#2980b9', linewidth=1.5)
                return []
            ani = FuncAnimation(fig, anim_frame, frames=n_frames, interval=50, blit=False)
            fname_gif = f'continuum_leg_{gait}.gif'
            ani.save(fname_gif, writer=PillowWriter(fps=20), dpi=100)
            print(f'Saved: {fname_gif}')
            try:
                fname_mp4 = f'continuum_leg_{gait}.mp4'
                ani.save(fname_mp4, writer='ffmpeg', fps=20, dpi=100)
                print(f'Saved: {fname_mp4}')
            except Exception:
                pass
        except Exception as e:
            print(f'Animation export failed: {e}')
    
    def _timer_cb():
        if radio.value_selected != 'Manual' and HAS_CPG:
            update()
    
    radio.on_clicked(on_gait_select)
    sl1.on_changed(lambda v: update() if radio.value_selected == 'Manual' else None)
    sl2.on_changed(lambda v: update() if radio.value_selected == 'Manual' else None)
    sl3.on_changed(lambda v: update() if radio.value_selected == 'Manual' else None)
    sl4.on_changed(lambda v: update() if radio.value_selected == 'Manual' else None)
    btn_screenshot.on_clicked(on_screenshot)
    btn_animate.on_clicked(on_animate)
    
    timer = fig.canvas.new_timer(interval=50)
    timer.add_callback(_timer_cb)
    if HAS_CPG:
        timer.start()
    
    update(None)
    plt.show()


if __name__ == '__main__':
    main()
