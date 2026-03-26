import sys
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle, Circle, Polygon
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patheffects as pe
import time
import requests
import io
from PIL import Image

# ==========================================
# CONFIGURATION
# ==========================================
ESP32_IP     = sys.argv[1] if len(sys.argv) > 1 else '192.168.1.201'
STREAM_URL   = f'http://{ESP32_IP}/stream'
IMU_URL      = f'http://{ESP32_IP}/api/imu'
SNAPSHOT_URL = f'http://{ESP32_IP}/api/snapshot'
MAX_DISTANCE = 2000  # mm
# ==========================================

# ==========================================
# DESIGN TOKENS
# ==========================================
BG_BASE     = '#05080F'
BG_CARD     = '#0B1525'
BORDER      = '#112240'
BLUE_DIM    = '#0D2545'
BLUE_MID    = '#1565C0'
BLUE_BRIGHT = '#1E90FF'
BLUE_GLOW   = '#42A5F5'
CYAN        = '#00D4FF'
CYAN_LABEL  = '#7DD3FC'   # bright-but-calm label blue — readable against dark cards
AMBER       = '#FFC107'
WHITE_SOFT  = '#CBD8E6'
WHITE_DIM   = '#5C7A9B'
RED_ALERT   = '#EF5350'

# ==========================================
# DATA
# ==========================================
grid_A   = np.ones((8, 8)) * MAX_DISTANCE
grid_B   = np.ones((8, 8)) * MAX_DISTANCE
imu_data = {'pitch': 0.0, 'roll': 0.0, 'heading': 0.0}
latest_frame = None
data_lock     = threading.Lock()
imu_lock      = threading.Lock()
frame_lock    = threading.Lock()
snapshot_lock = threading.Lock()

# ==========================================
# WIFI DATA THREADS
# ==========================================
def mjpeg_reader():
    """Pull MJPEG frames from /stream, store latest as numpy array."""
    global latest_frame
    backoff = 1
    while True:
        try:
            r = requests.get(STREAM_URL, stream=True, timeout=10)
            backoff = 1
            buf = b''
            for chunk in r.iter_content(chunk_size=4096):
                buf += chunk
                a = buf.find(b'\xff\xd8')
                b = buf.find(b'\xff\xd9')
                if a != -1 and b != -1 and b > a:
                    jpg = buf[a:b+2]
                    buf = buf[b+2:]
                    try:
                        img = Image.open(io.BytesIO(jpg))
                        arr = np.array(img)
                        with frame_lock:
                            latest_frame = arr
                    except Exception:
                        pass
        except Exception as e:
            print(f"MJPEG error: {e}")
            time.sleep(backoff)
            backoff = min(backoff * 2, 5)

def imu_poller():
    """Poll /api/imu at ~20Hz for fast heading updates."""
    backoff = 1
    while True:
        try:
            r = requests.get(IMU_URL, timeout=2)
            if r.status_code == 200:
                d = r.json()
                with imu_lock:
                    imu_data['pitch']   = d.get('pitch', 0.0)
                    imu_data['roll']    = d.get('roll', 0.0)
                    imu_data['heading'] = d.get('heading', 0.0)
                backoff = 1
            time.sleep(0.05)
        except Exception as e:
            print(f"IMU poll error: {e}")
            time.sleep(backoff)
            backoff = min(backoff * 2, 5)

def snapshot_poller():
    """Poll /api/snapshot at ~5Hz for synced ToF grids."""
    global grid_A, grid_B
    backoff = 1
    while True:
        try:
            r = requests.get(SNAPSHOT_URL, timeout=2)
            if r.status_code == 200:
                d = r.json()
                with snapshot_lock:
                    if d.get('tof_a') is not None:
                        grid_A = np.array(d['tof_a'])
                    if d.get('tof_b') is not None:
                        grid_B = np.array(d['tof_b'])
                backoff = 1
            time.sleep(0.2)
        except Exception as e:
            print(f"Snapshot poll error: {e}")
            time.sleep(backoff)
            backoff = min(backoff * 2, 5)

# ==========================================
# 3D BOX MATH
# ==========================================
w, l, h = 1.2, 2.5, 0.4
base_vertices = np.array([
    [-w/2, -l/2, -h/2], [w/2, -l/2, -h/2], [w/2,  l/2, -h/2], [-w/2,  l/2, -h/2],
    [-w/2, -l/2,  h/2], [w/2, -l/2,  h/2], [w/2,  l/2,  h/2], [-w/2,  l/2,  h/2],
])
box_edges = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

def get_rotation_matrix(pitch_deg, roll_deg, yaw_deg):
    # Pitch rotates around X-axis (width)
    # Roll rotates around Y-axis (length)
    p, r, y = np.radians(pitch_deg), np.radians(roll_deg), np.radians(yaw_deg)
    Rx = np.array([[1,0,0],[0,np.cos(p),-np.sin(p)],[0,np.sin(p),np.cos(p)]])
    Ry = np.array([[np.cos(r),0,np.sin(r)],[0,1,0],[-np.sin(r),0,np.cos(r)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

# ==========================================
# FIGURE
# ==========================================
plt.rcParams.update({
    'font.family':     'DejaVu Sans',
    'text.color':      WHITE_SOFT,
    'axes.labelcolor': WHITE_DIM,
    'xtick.color':     WHITE_DIM,
    'ytick.color':     WHITE_DIM,
})

fig = plt.figure(figsize=(22, 12), facecolor=BG_BASE)
fig.canvas.manager.set_window_title('AutoBoat  ·  Navigation & Proximity System')

# ── Title bar ────────────────────────────────────────────────────────────
fig.text(0.5, 0.973, 'AUTOBOAT  ·  NAVIGATION SYSTEM',
         ha='center', va='top', fontsize=15, fontweight='bold',
         color=CYAN, fontfamily='monospace',
         path_effects=[pe.withSimplePatchShadow(
             shadow_rgbFace=BLUE_MID, alpha=0.35, rho=0.6)])
fig.text(0.5, 0.946, '━' * 120,
         ha='center', va='top', fontsize=7, color=BLUE_DIM)

# ── 6-column, 2-row grid ─────────────────────────────────────────────────
#
#   Row 0  │  Sensor A  [cols 0:3]  │  Sensor B  [cols 3:6]  │
#   Row 1  │  IMU       [cols 0:2]  │  Compass   [cols 2:4]  │  3D  [cols 4:6]  │
#
#   Layout guarantees:
#     • IMU  left  edge == Sensor A left  edge  (col 0)
#     • 3D   right edge == Sensor B right edge  (col 6)
#     • Compass centre  == col-3 seam between A and B  ✓
#     • height_ratios slightly reduced for row 0 (less top-heavy)
# ─────────────────────────────────────────────────────────────────────────
gs = gridspec.GridSpec(
    3, 6, figure=fig,
    height_ratios=[1.65, 1.0, 1.2],
    hspace=0.44,
    wspace=0.30,
    left=0.04, right=0.97,
    top=0.916, bottom=0.04,
)

ax_A    = fig.add_subplot(gs[0, 0:3])
ax_B    = fig.add_subplot(gs[0, 3:6])
ax_imu  = fig.add_subplot(gs[1, 0:2])
ax_comp = fig.add_subplot(gs[1, 2:4])
ax_3d   = fig.add_subplot(gs[1, 4:6], projection='3d')
ax_cam  = fig.add_subplot(gs[2, :])

# ==========================================
# CARD STYLER
# ==========================================
def style_card(ax, title, title_color=CYAN):
    ax.set_facecolor(BG_CARD)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)
        sp.set_linewidth(1.2)
    ax.set_title(title, fontsize=11, color=title_color,
                 pad=9, fontweight='bold', loc='left',
                 fontfamily='monospace')

# ==========================================
# HEATMAPS  (Sensor A & B)
# ==========================================
CMAP = 'Blues_r'

for ax, title in [(ax_A, '◈  SENSOR A  ·  PORT'),
                  (ax_B, '◈  SENSOR B  ·  STARBOARD')]:
    style_card(ax, title)
    ax.set_xticks(range(8))
    ax.set_yticks(range(8))
    ax.set_xticklabels([str(i+1) for i in range(8)], fontsize=10, color=WHITE_SOFT)
    ax.set_yticklabels([str(i+1) for i in range(8)], fontsize=10, color=WHITE_SOFT)
    ax.tick_params(length=3, width=1.0)
    ax.grid(color=BORDER, linestyle='-', linewidth=0.5, alpha=0.5)

im1 = ax_A.imshow(grid_A, vmin=0, vmax=MAX_DISTANCE,
                   cmap=CMAP, interpolation='gaussian', aspect='auto')
im2 = ax_B.imshow(grid_B, vmin=0, vmax=MAX_DISTANCE,
                   cmap=CMAP, interpolation='gaussian', aspect='auto')

rect_a = Rectangle((-0.5,-0.5), 1, 1, fill=False,
                    edgecolor=CYAN, linewidth=2.0, linestyle='--', zorder=5)
rect_b = Rectangle((-0.5,-0.5), 1, 1, fill=False,
                    edgecolor=CYAN, linewidth=2.0, linestyle='--', zorder=5)
ax_A.add_patch(rect_a)
ax_B.add_patch(rect_b)

bbox_kw = dict(boxstyle='round,pad=0.3', facecolor=BG_BASE,
               edgecolor=BORDER, linewidth=0.8)
stats_a = ax_A.text(7.4, -0.6, '', color=CYAN, fontsize=9,
                     ha='right', va='top', fontfamily='monospace', bbox=bbox_kw)
stats_b = ax_B.text(7.4, -0.6, '', color=CYAN, fontsize=9,
                     ha='right', va='top', fontfamily='monospace', bbox=bbox_kw)

for im, ax in [(im1, ax_A), (im2, ax_B)]:
    cb = fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02)
    cb.ax.yaxis.set_tick_params(color=WHITE_SOFT, labelsize=9)
    cb.outline.set_edgecolor(BORDER)
    cb.set_label('mm', color=WHITE_SOFT, fontsize=10)
    plt.setp(cb.ax.yaxis.get_ticklabels(), color=WHITE_SOFT)

# ==========================================
# IMU PANEL  —  Pitch & Roll
# Two locked label+value units, centered in each half of the panel.
# No decorative bars, no floating rules — just clean paired data.
# ==========================================
ax_imu.set_facecolor(BG_CARD)
for sp in ax_imu.spines.values():
    sp.set_edgecolor(BORDER); sp.set_linewidth(1.2)
ax_imu.axis('off')
ax_imu.set_title('◈  ATTITUDE', fontsize=11, color=CYAN,
                  pad=9, fontweight='bold', loc='left', fontfamily='monospace')

# Each metric occupies one horizontal half of the panel.
# Label sits directly above the number — they are one unit.
#   PITCH block centred at x=0.28, ROLL block centred at x=0.72
for cx, label in [(0.28, 'PITCH'), (0.72, 'ROLL')]:
    # Dim, small label — contrast with the bright number below does all the work
    ax_imu.text(cx, 0.58, label,
                transform=ax_imu.transAxes, color=WHITE_DIM,
                fontsize=8, fontfamily='monospace', fontweight='normal',
                ha='center', va='bottom', alpha=0.75)
    # No underline — deleted entirely

val_pitch = ax_imu.text(
    0.28, 0.20, '+00.00°', transform=ax_imu.transAxes,
    color=BLUE_GLOW, fontsize=30, fontweight='bold',
    fontfamily='monospace', ha='center', va='bottom')

val_roll = ax_imu.text(
    0.72, 0.20, '+00.00°', transform=ax_imu.transAxes,
    color=BLUE_GLOW, fontsize=30, fontweight='bold',
    fontfamily='monospace', ha='center', va='bottom')

# ==========================================
# COMPASS  —  centred on the A | B seam
# Heading value sits directly under the dial
# ==========================================
ax_comp.set_facecolor(BG_CARD)
for sp in ax_comp.spines.values():
    sp.set_edgecolor(BORDER); sp.set_linewidth(1.2)
ax_comp.set_title('◈  COMPASS', fontsize=11, color=CYAN,
                   pad=9, fontweight='bold', loc='left', fontfamily='monospace')
ax_comp.axis('off')

COMP_R   = 0.80
TICK_OUT = 0.88
LABEL_R  = 1.03

# Bottom space reserved for large heading readout
ax_comp.set_xlim(-1.26, 1.26)
ax_comp.set_ylim(-1.55, 1.24)
ax_comp.set_aspect('equal')

# Background fill — solid base plate
for r, alpha in [(TICK_OUT + 0.04, 0.22), (COMP_R, 0.12)]:
    ax_comp.add_patch(Circle((0, 0), r, color=BLUE_MID, alpha=alpha, zorder=0))

# Radial vignette: stacked dark rings, opaque at rim fading clear at centre.
# Creates the illusion of a recessed, domed dial face.
_N_VIGNETTE = 28
for i in range(_N_VIGNETTE):
    # fraction 0 = rim, 1 = centre
    frac   = i / _N_VIGNETTE
    radius = COMP_R * (1.0 - frac * 0.90)
    # Alpha peaks at the rim (frac≈0), drops to 0 at centre (frac≈1)
    valpha = 0.055 * (1.0 - frac) ** 1.6
    ax_comp.add_patch(Circle((0, 0), radius,
                              color='#020510', alpha=valpha, zorder=1))

ax_comp.add_patch(Circle((0, 0), TICK_OUT + 0.04, fill=False,
                           color=BLUE_BRIGHT, linewidth=1.8, zorder=2, alpha=0.7))
ax_comp.add_patch(Circle((0, 0), COMP_R, fill=False,
                           color=BORDER, linewidth=0.8, zorder=2))

# Tick marks
for deg in range(0, 360, 5):
    a = np.radians(deg)
    sa, ca = np.sin(a), np.cos(a)
    if deg % 90 == 0:
        r_in, lw, col = COMP_R * 0.80, 2.0, CYAN
    elif deg % 45 == 0:
        r_in, lw, col = COMP_R * 0.85, 1.4, BLUE_GLOW
    elif deg % 10 == 0:
        r_in, lw, col = COMP_R * 0.91, 1.0, BLUE_MID
    else:
        r_in, lw, col = COMP_R * 0.955, 0.5, BORDER
    ax_comp.plot([sa * r_in, sa * TICK_OUT],
                 [ca * r_in, ca * TICK_OUT],
                 color=col, linewidth=lw, zorder=2)

# Cardinals
cardinals = {
    0:   ('N',  RED_ALERT,  13, 'bold'),
    90:  ('E',  WHITE_SOFT, 10, 'normal'),
    180: ('S',  WHITE_SOFT, 10, 'normal'),
    270: ('W',  WHITE_SOFT, 10, 'normal'),
    45:  ('NE', WHITE_DIM,   8, 'normal'),
    135: ('SE', WHITE_DIM,   8, 'normal'),
    225: ('SW', WHITE_DIM,   8, 'normal'),
    315: ('NW', WHITE_DIM,   8, 'normal'),
}
for deg, (lbl, col, fs, fw) in cardinals.items():
    a = np.radians(deg)
    ax_comp.text(np.sin(a) * LABEL_R, np.cos(a) * LABEL_R,
                 lbl, color=col, ha='center', va='center',
                 fontsize=fs, fontweight=fw, zorder=3, fontfamily='monospace')

# Hub
ax_comp.add_patch(Circle((0, 0), 0.09, color=BLUE_BRIGHT, alpha=0.5, zorder=6))
ax_comp.add_patch(Circle((0, 0), 0.045, color=CYAN, zorder=7))

# Needle
NEEDLE_TIP  = 0.66
NEEDLE_BASE = 0.22
NEEDLE_W    = 0.050

north_tri_pts = np.array([[0,  NEEDLE_TIP ], [-NEEDLE_W, 0], [NEEDLE_W, 0]])
south_tri_pts = np.array([[0, -NEEDLE_BASE], [-NEEDLE_W, 0], [NEEDLE_W, 0]])

needle_north = Polygon(north_tri_pts, closed=True,
                        facecolor=RED_ALERT, edgecolor='#FF8A80',
                        linewidth=0.8, zorder=5,
                        path_effects=[pe.withSimplePatchShadow(
                            shadow_rgbFace='#B71C1C', alpha=0.6, rho=0.7)])
needle_south = Polygon(south_tri_pts, closed=True,
                        facecolor=BLUE_DIM, edgecolor=BORDER,
                        linewidth=0.8, zorder=5)
ax_comp.add_patch(needle_north)
ax_comp.add_patch(needle_south)

# ── Heading value — large, directly beneath the dial ──────────────────────
compass_hdg_val = ax_comp.text(
    0, -1.28, '000.0°',
    color=AMBER, ha='center', va='center',
    fontsize=24, fontweight='bold',
    fontfamily='monospace', zorder=6)
# "HEADING" label removed — the degree symbol, amber colour and compass above
# already make this self-evident; the blank space reads as intentional.

def rotate_needle_pts(base_pts, heading_deg):
    a = np.radians(-heading_deg)
    c, s = np.cos(a), np.sin(a)
    R2 = np.array([[c, -s], [s, c]])
    return (R2 @ base_pts.T).T

# ==========================================
# 3D ORIENTATION
# ==========================================
# Transparent axes background — boat floats on pure black like a HUD element
ax_3d.set_facecolor('none')
ax_3d.patch.set_alpha(0.0)
ax_3d.set_title('◈  3D ORIENTATION', fontsize=11, color=CYAN,
                 pad=9, fontweight='bold', loc='left', fontfamily='monospace')

# Symmetric limits: (0,0,0) sits at dead centre of the bounding box,
# so the boat pivots cleanly around its own centre on every axis.
ax_3d.set_xlim([-2, 2]); ax_3d.set_ylim([-2, 2]); ax_3d.set_zlim([-2, 2])

# Isometric camera — angled enough to see pitch, roll & yaw simultaneously
ax_3d.view_init(elev=22, azim=-55)

# Grid: only halves (-2, 0, 2) — two divisions per axis, no subdivision clutter
ax_3d.set_xticks([-2, 0, 2])
ax_3d.set_yticks([-2, 0, 2])
ax_3d.set_zticks([-2, 0, 2])

# No tick labels or marks
ax_3d.set_xticklabels([])
ax_3d.set_yticklabels([])
ax_3d.set_zticklabels([])
ax_3d.tick_params(axis='both', which='both', length=0, pad=0)

# Axis name labels
ax_3d.set_xlabel('X', color=CYAN, fontsize=11, fontweight='bold', labelpad=1)
ax_3d.set_ylabel('Y', color=CYAN, fontsize=11, fontweight='bold', labelpad=1)
ax_3d.set_zlabel('Z', color=CYAN, fontsize=11, fontweight='bold', labelpad=1)

# Kill all pane fills AND pane edges — no grey/coloured background panels at all
ax_3d.xaxis.pane.fill = False
ax_3d.yaxis.pane.fill = False
ax_3d.zaxis.pane.fill = False
ax_3d.xaxis.pane.set_edgecolor('none')
ax_3d.yaxis.pane.set_edgecolor('none')
ax_3d.zaxis.pane.set_edgecolor('none')

# Sparse grid lines in the same dark border tone as the rest of the UI
ax_3d.grid(True, color=BORDER, linewidth=0.5, alpha=0.5)

# Box wireframe edges
box_lines = [ax_3d.plot([], [], [], color=BLUE_BRIGHT,
                         linewidth=2.0, alpha=0.9)[0] for _ in range(12)]
fwd_line, = ax_3d.plot([], [], [], color=RED_ALERT,
                        linewidth=3.0, marker='^', markersize=8, alpha=1.0)

# Box face fills — initialized to zero-size quads so no artifact appears
# before the first animation frame writes real rotated vertices.
_zero_face = [[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]]
box_faces = Poly3DCollection(
    _zero_face * 6,
    facecolor=BLUE_MID,
    edgecolor='none',
    alpha=0.17,
    zorder=2
)
ax_3d.add_collection3d(box_faces)

# ==========================================
# CAMERA PANEL
# ==========================================
style_card(ax_cam, '◈  CAMERA  ·  LIVE FEED')
ax_cam.axis('off')
# Placeholder image — black until first frame arrives
cam_placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
cam_img = ax_cam.imshow(cam_placeholder, aspect='auto')
cam_no_signal = ax_cam.text(
    0.5, 0.5, 'NO CAMERA', transform=ax_cam.transAxes,
    color=WHITE_DIM, fontsize=18, fontfamily='monospace',
    ha='center', va='center', alpha=0.6)

# ==========================================
# BOTTOM ROW DIVIDERS  (drawn once in figure coords)
# Two faint vertical lines with alpha that fades to 0 at top & bottom —
# soft gradient separators, not rigid hard grid lines.
# ==========================================
_dividers_drawn = [False]

def draw_bottom_dividers():
    if _dividers_drawn[0]:
        return
    _dividers_drawn[0] = True

    pos_imu  = ax_imu.get_position()
    pos_comp = ax_comp.get_position()
    pos_3d   = ax_3d.get_position()
    y0 = min(pos_imu.y0, pos_comp.y0, pos_3d.y0)
    y1 = max(pos_imu.y1, pos_comp.y1, pos_3d.y1)
    x_div1 = (pos_imu.x1 + pos_comp.x0) / 2
    x_div2 = (pos_comp.x1 + pos_3d.x0)  / 2

    # Simulate a gradient by drawing N small segments with varying alpha.
    # Alpha envelope: sin²(t) peaks at centre (t=π/2) and is 0 at ends.
    N = 60
    ys = np.linspace(y0, y1, N + 1)
    for xd in (x_div1, x_div2):
        for i in range(N):
            t     = np.pi * i / (N - 1)          # 0 → π
            alpha = (np.sin(t) ** 2) * 0.55       # 0 at ends, 0.55 at centre
            fig.add_artist(plt.Line2D(
                [xd, xd], [ys[i], ys[i+1]],
                transform=fig.transFigure,
                color=BLUE_MID, linewidth=1.2,
                alpha=float(alpha), zorder=0,
                solid_capstyle='butt'
            ))

# ==========================================
# ANIMATION UPDATE
# ==========================================
# Face index quads — defined here so update_plot can reference them
BOX_FACE_QUADS = [
    (0, 1, 2, 3),   # bottom
    (4, 5, 6, 7),   # top
    (0, 1, 5, 4),   # front
    (2, 3, 7, 6),   # back
    (0, 3, 7, 4),   # left
    (1, 2, 6, 5),   # right
]

def update_plot(frame):
    draw_bottom_dividers()

    with snapshot_lock:
        gA  = np.copy(grid_A)
        gB  = np.copy(grid_B)
    with imu_lock:
        p   = imu_data['pitch']
        r   = imu_data['roll']
        hdg = imu_data['heading']

    # Camera frame update
    with frame_lock:
        cam_frame = latest_frame
    if cam_frame is not None:
        cam_img.set_data(cam_frame)
        cam_no_signal.set_visible(False)
    else:
        cam_no_signal.set_visible(True)

    # 1 — Heatmaps
    for grid, im, rect, stats in [
        (gA, im1, rect_a, stats_a),
        (gB, im2, rect_b, stats_b),
    ]:
        im.set_data(grid)
        min_val = int(np.min(grid))
        avg_val = int(np.mean(grid))
        idx = np.unravel_index(np.argmin(grid), grid.shape)
        rect.set_xy((idx[1] - 0.5, idx[0] - 0.5))
        stats.set_text(f'MIN {min_val:4d}  AVG {avg_val:4d}  mm')

    # 2 — Pitch & Roll  (colour by magnitude)
    def att_color(v):
        return RED_ALERT if abs(v) > 20 else AMBER if abs(v) > 10 else BLUE_GLOW

    val_pitch.set_text(f'{p:+.2f}°')
    val_pitch.set_color(att_color(p))
    val_roll.set_text(f'{r:+.2f}°')
    val_roll.set_color(att_color(r))

    # 3 — Compass + Heading value below
    needle_north.set_xy(rotate_needle_pts(north_tri_pts, hdg))
    needle_south.set_xy(rotate_needle_pts(south_tri_pts, hdg))
    compass_hdg_val.set_text(f'{hdg % 360:05.1f}°')

    # 4 — 3D box: pitch & roll ONLY — yaw is handled exclusively by the compass.
    # Passing 0 for yaw locks the bow arrow to always point the same direction;
    # the box only tilts, never spins, making balance instantly readable.
    R  = get_rotation_matrix(p, r, 0)
    rv = (R @ base_vertices.T).T

    for line, edge in zip(box_lines, box_edges):
        p1, p2 = rv[edge[0]], rv[edge[1]]
        line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        line.set_3d_properties([p1[2], p2[2]])
        
    # Draw arrow along the centre spine: from back (-l/2) to nose (+l/2 + tip)
    arrow_base = np.array([0, -l/2, 0])
    arrow_tip  = np.array([0,  l/2 + 0.5, 0])

    arr_b_rot = R @ arrow_base
    arr_t_rot = R @ arrow_tip

    fwd_line.set_data([arr_b_rot[0], arr_t_rot[0]], [arr_b_rot[1], arr_t_rot[1]])
    fwd_line.set_3d_properties([arr_b_rot[2], arr_t_rot[2]])

    # Update face fills
    new_faces = [[rv[i].tolist() for i in quad] for quad in BOX_FACE_QUADS]
    box_faces.set_verts(new_faces)

    return ([im1, im2, rect_a, rect_b, stats_a, stats_b,
              val_pitch, val_roll,
              needle_north, needle_south, compass_hdg_val,
              fwd_line, box_faces, cam_img, cam_no_signal]
            + box_lines)

# ==========================================
# MAIN
# ==========================================
if __name__ == '__main__':
    print(f'Connecting to ESP32 at {ESP32_IP}...')
    print(f'  MJPEG:    {STREAM_URL}')
    print(f'  IMU:      {IMU_URL}')
    print(f'  Snapshot: {SNAPSHOT_URL}')

    for target in [mjpeg_reader, imu_poller, snapshot_poller]:
        threading.Thread(target=target, daemon=True).start()

    print('AutoBoat visualizer running... (close window to stop)')
    ani = animation.FuncAnimation(
        fig, update_plot,
        interval=20,
        blit=True,
        cache_frame_data=False
    )
    plt.show()