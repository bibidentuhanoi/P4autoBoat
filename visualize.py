# import serial
# import threading
# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.patches import Rectangle
# import time
# import re

# # ==========================================
# # CONFIGURATION
# # ==========================================
# COM_PORT = '/dev/ttyACM0'
# BAUD_RATE = 115200
# MAX_DISTANCE = 2000  # mm (The color scale limit)
# # ==========================================

# # Data structures
# grid_A = np.ones((8, 8)) * MAX_DISTANCE
# grid_B = np.ones((8, 8)) * MAX_DISTANCE
# data_lock = threading.Lock()

# def serial_reader():
#     global grid_A, grid_B
#     while True:
#         try:
#             ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
#             ser.reset_input_buffer()
            
#             current_sensor = None
#             temp_grid = []

#             while True:
#                 line_raw = ser.readline()
#                 if not line_raw: continue
#                 try:
#                     line = line_raw.decode('utf-8', errors='ignore').strip()
#                 except: continue

#                 if "SENSOR A" in line:
#                     current_sensor = 'A'; temp_grid = []
#                     continue
#                 elif "SENSOR B" in line:
#                     current_sensor = 'B'; temp_grid = []
#                     continue

#                 nums = [int(s) for s in re.findall(r'\d+', line)]
#                 if len(nums) == 8 and current_sensor is not None:
#                     temp_grid.append(nums)
#                     if len(temp_grid) == 8:
#                         with data_lock:
#                             if current_sensor == 'A': grid_A = np.array(temp_grid)
#                             else: grid_B = np.array(temp_grid)
#                         temp_grid = []
#                         current_sensor = None 
#         except:
#             time.sleep(2)

# # ------------------------------------------
# # Setup UI
# # ------------------------------------------
# plt.style.use('dark_background')
# fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
# fig.canvas.manager.set_window_title('VL53L5CX Dual 8x8 Live Radar')

# cmap = 'turbo' 

# # Sensor A Setup
# im1 = ax1.imshow(grid_A, vmin=0, vmax=MAX_DISTANCE, cmap=cmap, interpolation='gaussian')
# ax1.set_title("SENSOR A (Left)", fontsize=16, color='#00FFCC', pad=20)
# target_a = ax1.text(0, -1, "", color='white', fontsize=12, fontweight='bold')
# rect_a = Rectangle((0,0), 1, 1, fill=False, color='white', linewidth=2)
# ax1.add_patch(rect_a)

# # Sensor B Setup
# im2 = ax2.imshow(grid_B, vmin=0, vmax=MAX_DISTANCE, cmap=cmap, interpolation='gaussian')
# ax2.set_title("SENSOR B (Right)", fontsize=16, color='#00FFCC', pad=20)
# target_b = ax2.text(0, -1, "", color='white', fontsize=12, fontweight='bold')
# rect_b = Rectangle((0,0), 1, 1, fill=False, color='white', linewidth=2)
# ax2.add_patch(rect_b)

# # Stats Text (at the bottom)
# stats_a = ax1.text(0, 8.5, "Min: ---mm | Avg: ---mm", color='yellow', fontsize=10)
# stats_b = ax2.text(0, 8.5, "Min: ---mm | Avg: ---mm", color='yellow', fontsize=10)

# for ax in [ax1, ax2]:
#     ax.set_xticks(range(8))
#     ax.set_yticks(range(8))
#     ax.grid(color='white', linestyle='--', linewidth=0.5, alpha=0.3)

# plt.colorbar(im1, ax=ax1, label='Distance (mm)', fraction=0.046, pad=0.04)
# plt.colorbar(im2, ax=ax2, label='Distance (mm)', fraction=0.046, pad=0.04)

# def update_plot(frame):
#     with data_lock:
#         # Update A
#         im1.set_data(grid_A)
#         min_a = np.min(grid_A)
#         avg_a = np.mean(grid_A)
#         min_idx_a = np.unravel_index(np.argmin(grid_A, axis=None), grid_A.shape)
#         rect_a.set_xy((min_idx_a[1]-0.5, min_idx_a[0]-0.5))
#         stats_a.set_text(f"CLOSEST: {min_a}mm | AVG: {int(avg_a)}mm")
        
#         # Update B
#         im2.set_data(grid_B)
#         min_b = np.min(grid_B)
#         avg_b = np.mean(grid_B)
#         min_idx_b = np.unravel_index(np.argmin(grid_B, axis=None), grid_B.shape)
#         rect_b.set_xy((min_idx_b[1]-0.5, min_idx_b[0]-0.5))
#         stats_b.set_text(f"CLOSEST: {min_b}mm | AVG: {int(avg_b)}mm")

#     return [im1, im2, rect_a, rect_b, stats_a, stats_b]

# if __name__ == '__main__':
#     thread = threading.Thread(target=serial_reader, daemon=True)
#     thread.start()

#     print("Visualizer running...")
#     ani = animation.FuncAnimation(fig, update_plot, interval=30, blit=True, cache_frame_data=False)
#     plt.tight_layout()
#     plt.show()
import serial
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle, FancyArrowPatch
import time
import re

# ==========================================
# CONFIGURATION
# ==========================================
COM_PORT     = '/dev/ttyACM0'
BAUD_RATE    = 115200
MAX_DISTANCE = 2000  # mm
# ==========================================

# Data structures
grid_A = np.ones((8, 8)) * MAX_DISTANCE
grid_B = np.ones((8, 8)) * MAX_DISTANCE

imu = {
    'ax': 0.0, 'ay': 0.0, 'az': 0.0,
    'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
    'mx': 0.0, 'my': 0.0, 'mz': 0.0,
}

data_lock = threading.Lock()

# ==========================================
# SERIAL READER THREAD
# ==========================================
IMU_PATTERN = re.compile(
    r'AX:([-\d.]+)\s+AY:([-\d.]+)\s+AZ:([-\d.]+)\s+'
    r'GX:([-\d.]+)\s+GY:([-\d.]+)\s+GZ:([-\d.]+)\s+'
    r'MX:([-\d.]+)\s+MY:([-\d.]+)\s+MZ:([-\d.]+)'
)

def serial_reader():
    global grid_A, grid_B, imu
    while True:
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
            ser.reset_input_buffer()

            current_sensor = None
            temp_grid = []

            while True:
                line_raw = ser.readline()
                if not line_raw:
                    continue
                try:
                    line = line_raw.decode('utf-8', errors='ignore').strip()
                except:
                    continue

                # ToF sensor header detection
                if "SENSOR A" in line:
                    current_sensor = 'A'
                    temp_grid = []
                    continue
                elif "SENSOR B" in line:
                    current_sensor = 'B'
                    temp_grid = []
                    continue

                # IMU line detection
                if line.startswith("IMU:"):
                    m = IMU_PATTERN.search(line)
                    if m:
                        vals = [float(x) for x in m.groups()]
                        with data_lock:
                            imu['ax'], imu['ay'], imu['az'] = vals[0], vals[1], vals[2]
                            imu['gx'], imu['gy'], imu['gz'] = vals[3], vals[4], vals[5]
                            imu['mx'], imu['my'], imu['mz'] = vals[6], vals[7], vals[8]
                    continue

                # ToF grid rows
                nums = [int(s) for s in re.findall(r'\d+', line)]
                if len(nums) == 8 and current_sensor is not None:
                    temp_grid.append(nums)
                    if len(temp_grid) == 8:
                        with data_lock:
                            if current_sensor == 'A':
                                grid_A = np.array(temp_grid)
                            else:
                                grid_B = np.array(temp_grid)
                        temp_grid = []
                        current_sensor = None

        except Exception as e:
            print(f"Serial error: {e}")
            time.sleep(2)

# ==========================================
# UI SETUP
# ==========================================
plt.style.use('dark_background')

# Layout: top row = two heatmaps, bottom = IMU panel
fig = plt.figure(figsize=(16, 10))
fig.canvas.manager.set_window_title('VL53L5CX Dual 8x8 + ICM-20948 Live View')

gs = gridspec.GridSpec(2, 3, figure=fig,
                       height_ratios=[2.2, 1],
                       hspace=0.45, wspace=0.35)

ax1   = fig.add_subplot(gs[0, 0])   # Sensor A heatmap
ax2   = fig.add_subplot(gs[0, 2])   # Sensor B heatmap
ax_ag = fig.add_subplot(gs[1, 0])   # Accel + Gyro bars
ax_m  = fig.add_subplot(gs[1, 1])   # Magnetometer compass
ax_v  = fig.add_subplot(gs[1, 2])   # Numerical readout

cmap = 'turbo'

# --- Sensor A ---
im1 = ax1.imshow(grid_A, vmin=0, vmax=MAX_DISTANCE, cmap=cmap, interpolation='gaussian')
ax1.set_title("SENSOR A (Left)", fontsize=14, color='#00FFCC', pad=12)
rect_a = Rectangle((-0.5, -0.5), 1, 1, fill=False, color='white', linewidth=2)
ax1.add_patch(rect_a)
stats_a = ax1.text(0, 8.6, "CLOSEST: ---mm | AVG: ---mm", color='yellow', fontsize=9)
for ax in [ax1]:
    ax.set_xticks(range(8)); ax.set_yticks(range(8))
    ax.grid(color='white', linestyle='--', linewidth=0.4, alpha=0.3)
plt.colorbar(im1, ax=ax1, label='Distance (mm)', fraction=0.046, pad=0.04)

# --- Sensor B ---
im2 = ax2.imshow(grid_B, vmin=0, vmax=MAX_DISTANCE, cmap=cmap, interpolation='gaussian')
ax2.set_title("SENSOR B (Right)", fontsize=14, color='#00FFCC', pad=12)
rect_b = Rectangle((-0.5, -0.5), 1, 1, fill=False, color='white', linewidth=2)
ax2.add_patch(rect_b)
stats_b = ax2.text(0, 8.6, "CLOSEST: ---mm | AVG: ---mm", color='yellow', fontsize=9)
for ax in [ax2]:
    ax.set_xticks(range(8)); ax.set_yticks(range(8))
    ax.grid(color='white', linestyle='--', linewidth=0.4, alpha=0.3)
plt.colorbar(im2, ax=ax2, label='Distance (mm)', fraction=0.046, pad=0.04)

# --- Accel + Gyro Bar Chart ---
ax_ag.set_title("Accel (g) | Gyro (dps)", fontsize=11, color='#FF9944')
bar_labels  = ['AX', 'AY', 'AZ', 'GX', 'GY', 'GZ']
bar_colors  = ['#FF4444', '#44FF44', '#4444FF', '#FF8800', '#00CCFF', '#FF00FF']
bar_vals    = [0.0] * 6
bars        = ax_ag.bar(bar_labels, bar_vals, color=bar_colors, width=0.6)
ax_ag.set_ylim(-20, 20)
ax_ag.axhline(0, color='white', linewidth=0.5, alpha=0.5)
ax_ag.set_ylabel("Value", fontsize=9)
ax_ag.tick_params(labelsize=8)
accel_range_line = ax_ag.axhspan(-2, 2, alpha=0.05, color='red')   # visual accel range hint

# --- Magnetometer Compass ---
ax_m.set_xlim(-1.3, 1.3)
ax_m.set_ylim(-1.3, 1.3)
ax_m.set_aspect('equal')
ax_m.set_title("Magnetometer (XY)", fontsize=11, color='#FF9944')
ax_m.set_facecolor('#0a0a0a')
compass_circle = plt.Circle((0, 0), 1.0, color='#333333', fill=False, linewidth=1.5)
ax_m.add_patch(compass_circle)
for label, pos in [('N', (0, 1.15)), ('S', (0, -1.25)),
                    ('E', (1.15, 0)), ('W', (-1.25, 0))]:
    ax_m.text(pos[0], pos[1], label, ha='center', va='center',
              color='#888888', fontsize=9)
# Compass needle
mag_arrow, = ax_m.plot([0, 0], [0, 0.8], color='#FF4444', linewidth=3)
mag_dot    = ax_m.plot(0, 0, 'wo', markersize=5)[0]
ax_m.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
mag_strength_text = ax_m.text(0, -1.2, "Strength: --- uT", ha='center',
                               color='#AAAAAA', fontsize=8)

# --- Numerical Readout Panel ---
ax_v.axis('off')
ax_v.set_title("IMU Values", fontsize=11, color='#FF9944')
readout_text = ax_v.text(0.05, 0.95, "Waiting for data...",
                          transform=ax_v.transAxes,
                          color='white', fontsize=10,
                          verticalalignment='top',
                          fontfamily='monospace')

# ==========================================
# ANIMATION UPDATE
# ==========================================
def update_plot(frame):
    with data_lock:
        # --- Sensor A ---
        im1.set_data(grid_A)
        min_a     = np.min(grid_A)
        avg_a     = np.mean(grid_A)
        idx_a     = np.unravel_index(np.argmin(grid_A), grid_A.shape)
        rect_a.set_xy((idx_a[1] - 0.5, idx_a[0] - 0.5))
        stats_a.set_text(f"CLOSEST: {int(min_a)}mm | AVG: {int(avg_a)}mm")

        # --- Sensor B ---
        im2.set_data(grid_B)
        min_b     = np.min(grid_B)
        avg_b     = np.mean(grid_B)
        idx_b     = np.unravel_index(np.argmin(grid_B), grid_B.shape)
        rect_b.set_xy((idx_b[1] - 0.5, idx_b[0] - 0.5))
        stats_b.set_text(f"CLOSEST: {int(min_b)}mm | AVG: {int(avg_b)}mm")

        # --- Accel + Gyro Bars ---
        new_vals = [
            imu['ax'], imu['ay'], imu['az'],
            imu['gx'] / 100.0,  # Scale gyro to fit same axis (÷100 dps)
            imu['gy'] / 100.0,
            imu['gz'] / 100.0,
        ]
        for bar, val in zip(bars, new_vals):
            bar.set_height(val)
            # Clip bars to ylim to avoid rendering outside
            if val > 0:
                bar.set_y(0)
            else:
                bar.set_y(val)
                bar.set_height(abs(val))

        # --- Magnetometer Compass ---
        mx, my = imu['mx'], imu['my']
        strength = (mx**2 + my**2 + imu['mz']**2) ** 0.5
        norm = (mx**2 + my**2) ** 0.5
        if norm > 0:
            nx, ny = mx / norm * 0.9, my / norm * 0.9
        else:
            nx, ny = 0, 0.9
        mag_arrow.set_data([0, nx], [0, ny])
        mag_strength_text.set_text(f"Strength: {strength:.1f} uT")

        # --- Numerical Readout ---
        readout = (
            f"  ACCELEROMETER\n"
            f"  AX: {imu['ax']:+.3f} g\n"
            f"  AY: {imu['ay']:+.3f} g\n"
            f"  AZ: {imu['az']:+.3f} g\n\n"
            f"  GYROSCOPE\n"
            f"  GX: {imu['gx']:+7.2f} dps\n"
            f"  GY: {imu['gy']:+7.2f} dps\n"
            f"  GZ: {imu['gz']:+7.2f} dps\n\n"
            f"  MAGNETOMETER\n"
            f"  MX: {imu['mx']:+7.2f} uT\n"
            f"  MY: {imu['my']:+7.2f} uT\n"
            f"  MZ: {imu['mz']:+7.2f} uT\n"
            f"  |B|: {strength:.1f} uT"
        )
        readout_text.set_text(readout)

    return [im1, im2, rect_a, rect_b, stats_a, stats_b,
            *bars, mag_arrow, mag_strength_text, readout_text]


# ==========================================
# MAIN
# ==========================================
if __name__ == '__main__':
    thread = threading.Thread(target=serial_reader, daemon=True)
    thread.start()

    print("Visualizer running... (Close the window to stop)")
    ani = animation.FuncAnimation(
        fig, update_plot,
        interval=30,
        blit=True,
        cache_frame_data=False
    )
    plt.tight_layout()
    plt.show()