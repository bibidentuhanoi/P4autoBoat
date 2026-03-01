import sys
import serial
import threading
import numpy as np
import re
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QGridLayout, QGroupBox)
from PyQt6.QtCore import pyqtSignal, QObject, pyqtSlot
from PyQt6.QtGui import QFont, QColor
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# ==========================================
# CONFIGURATION
# ==========================================
COM_PORT     = '/dev/ttyACM0'
BAUD_RATE    = 115200
MAX_DISTANCE = 2000  # mm
# ==========================================

# Regex for parsing the ESP32 output:
# Example: [SENSOR A] IMU: Pitch:  -0.89 | Roll:   1.23 | Head: 135.45
IMU_PATTERN = re.compile(
    r'IMU:\s*Pitch:\s*([-\d.]+)\s*\|\s*Roll:\s*([-\d.]+)\s*\|\s*Head:\s*([-\d.]+)'
)

# ==========================================
# SERIAL READER (QThread-like approach)
# ==========================================
class SerialReader(QObject):
    # Signals to update GUI thread safely
    grid_updated = pyqtSignal(str, np.ndarray)  # sensor_id ('A' or 'B'), 8x8 grid
    imu_updated = pyqtSignal(float, float, float)  # pitch, roll, heading

    def __init__(self, port, baud, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False

    def _read_loop(self):
        ser = None
        while self.running:
            try:
                if ser is None or not ser.is_open:
                    ser = serial.Serial(self.port, self.baud, timeout=0.1)
                    ser.reset_input_buffer()
                    print(f"Connected to {self.port} at {self.baud}")

                current_sensor = None
                temp_grid = []

                while self.running:
                    try:
                        line_raw = ser.readline()
                        if not line_raw:
                            continue
                        line = line_raw.decode('utf-8', errors='ignore').strip()
                    except serial.SerialException:
                        break # Reconnect

                    # Parse IMU Data
                    if "IMU:" in line:
                        m = IMU_PATTERN.search(line)
                        if m:
                            p, r, h = map(float, m.groups())
                            self.imu_updated.emit(p, r, h)

                    # Determine active sensor for the grid
                    if "SENSOR A" in line:
                        current_sensor = 'A'
                        temp_grid = []
                        continue
                    elif "SENSOR B" in line:
                        current_sensor = 'B'
                        temp_grid = []
                        continue

                    # Parse 8x8 Grid rows
                    nums = [int(s) for s in re.findall(r'\d+', line)]
                    if len(nums) == 8 and current_sensor is not None:
                        temp_grid.append(nums)
                        if len(temp_grid) == 8:
                            grid_arr = np.array(temp_grid, dtype=float)
                            self.grid_updated.emit(current_sensor, grid_arr)
                            temp_grid = []
                            current_sensor = None

            except Exception as e:
                print(f"Serial connection error: {e}")
                time.sleep(2)

        if ser and ser.is_open:
            ser.close()

# ==========================================
# MAIN WINDOW
# ==========================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VL53L5CX & IMU 3D Visualizer")
        self.resize(1200, 800)

        # Main Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Top section: ToF Heatmaps
        top_layout = QHBoxLayout()
        main_layout.addLayout(top_layout, stretch=2)

        # Setup Sensor A Heatmap
        self.view_a = pg.ImageView()
        self.view_a.ui.histogram.hide()
        self.view_a.ui.roiBtn.hide()
        self.view_a.ui.menuBtn.hide()
        self.view_a.setPredefinedGradient('turbo')
        top_layout.addWidget(self._create_group("SENSOR A (Left)", self.view_a))

        # Setup Sensor B Heatmap
        self.view_b = pg.ImageView()
        self.view_b.ui.histogram.hide()
        self.view_b.ui.roiBtn.hide()
        self.view_b.ui.menuBtn.hide()
        self.view_b.setPredefinedGradient('turbo')
        top_layout.addWidget(self._create_group("SENSOR B (Right)", self.view_b))

        # Bottom section: 3D IMU & Text
        bottom_layout = QHBoxLayout()
        main_layout.addLayout(bottom_layout, stretch=1)

        # 3D IMU Widget
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 20
        self.gl_widget.opts['elevation'] = 30
        self.gl_widget.opts['azimuth'] = 45
        bottom_layout.addWidget(self._create_group("IMU 3D Orientation", self.gl_widget), stretch=2)

        # Add 3D elements
        grid = gl.GLGridItem()
        grid.scale(2, 2, 2)
        self.gl_widget.addItem(grid)

        # Axes: X=Red, Y=Green, Z=Blue
        axis = gl.GLAxisItem()
        axis.setSize(x=5, y=5, z=5)
        self.gl_widget.addItem(axis)

        # 3D Box representing the board
        verts = np.array([
            [ 2,  3,  0.2], [-2,  3,  0.2], [-2, -3,  0.2], [ 2, -3,  0.2], # Top
            [ 2,  3, -0.2], [-2,  3, -0.2], [-2, -3, -0.2], [ 2, -3, -0.2]  # Bottom
        ])
        faces = np.array([
            [0, 1, 2], [0, 2, 3], # Top
            [4, 5, 6], [4, 6, 7], # Bottom
            [0, 1, 5], [0, 5, 4], # Front
            [2, 3, 7], [2, 7, 6], # Back
            [1, 2, 6], [1, 6, 5], # Left
            [0, 3, 7], [0, 7, 4]  # Right
        ])
        colors = np.array([[0.2, 0.6, 1.0, 0.8] for _ in range(12)])
        self.box = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False, drawEdges=True, edgeColor=(1,1,1,1))
        self.gl_widget.addItem(self.box)

        # Text Stats
        self.lbl_stats = QLabel("Waiting for data...")
        self.lbl_stats.setFont(QFont("Monospace", 14))
        self.lbl_stats.setStyleSheet("color: #FF9944; background-color: #111; padding: 10px; border-radius: 5px;")
        bottom_layout.addWidget(self.lbl_stats, stretch=1)

        # Start Serial Reader
        self.reader = SerialReader(COM_PORT, BAUD_RATE)
        self.reader.grid_updated.connect(self.update_grid)
        self.reader.imu_updated.connect(self.update_imu)
        self.reader.start()

    def _create_group(self, title, widget):
        group = QGroupBox(title)
        group.setStyleSheet("QGroupBox { font-weight: bold; color: #00FFCC; }")
        layout = QVBoxLayout()
        layout.addWidget(widget)
        group.setLayout(layout)
        return group

    @pyqtSlot(str, np.ndarray)
    def update_grid(self, sensor_id, grid_data):
        # Clip max distance for better visualization contrast
        grid_data = np.clip(grid_data, 0, MAX_DISTANCE)
        # Flip vertically to match physical layout
        grid_data = np.flipud(grid_data)

        if sensor_id == 'A':
            self.view_a.setImage(grid_data.T, autoRange=False, autoLevels=False, levels=(0, MAX_DISTANCE))
        elif sensor_id == 'B':
            self.view_b.setImage(grid_data.T, autoRange=False, autoLevels=False, levels=(0, MAX_DISTANCE))

    @pyqtSlot(float, float, float)
    def update_imu(self, pitch, roll, heading):
        # Update text
        self.lbl_stats.setText(
            f"FUSION DATA\n"
            f"-----------\n"
            f"PITCH:   {pitch:+7.2f}°\n"
            f"ROLL:    {roll:+7.2f}°\n"
            f"HEADING: {heading:+7.2f}°"
        )

        # Update 3D Box Rotation
        # Reset transform
        self.box.resetTransform()

        # Apply Rotations
        # Pyqtgraph opengl rotations are (angle_deg, x, y, z)
        # Assuming Y is forward (heading/yaw), X is right (pitch), Z is up (roll) - Note: depends on sensor frame!
        # Standard aerospace sequence is Z (yaw), Y (pitch), X (roll).
        self.box.rotate(-heading, 0, 0, 1) # Yaw around Z (up)
        self.box.rotate(pitch, 1, 0, 0)   # Pitch around X (right)
        self.box.rotate(roll, 0, 1, 0)    # Roll around Y (forward)

    def closeEvent(self, event):
        self.reader.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # Set dark theme for the app
    app.setStyle('Fusion')
    palette = app.palette()
    palette.setColor(palette.ColorRole.Window, QColor(30, 30, 30))
    palette.setColor(palette.ColorRole.WindowText, QColor(255, 255, 255))
    app.setPalette(palette)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())
