"""
Robot Arm Controller — 3-axis serial interface with live 3D visualization.

Serial Protocol (115200 baud, newline-terminated)
=================================================
Commands TO Arduino:
    CMD:MOVE_ABS <α> <β> <γ> <speed>   – absolute move (degrees, °/s)
    CMD:MOVE_REL <α> <β> <γ> <speed>   – relative move
    CMD:STOP                             – emergency stop
    CMD:HOME <speed>                     – zero all axes
    CMD:QUERY                            – request current position

Responses FROM Arduino:
    POS:<α>,<β>,<γ>                      – current position (sent ~20 Hz)
    ACK:<echoed_command>                 – command acknowledged
    ERR:<message>                        – error
    DONE                                 – motion complete

Angle ranges:
    α (alpha) : 0–360°  XY-plane rotation (azimuth)
    β (beta)  : 0–180°  elevation from +Z axis
    γ (gamma) : 0–360°  end-effector roll about arm axis
"""

from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QColor, QIcon, QAction
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QDoubleSpinBox,
    QSlider, QTextEdit, QFileDialog, QMessageBox, QSplitter,
    QStatusBar, QFrame, QGridLayout, QSizePolicy, QProgressBar,
)

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Arm constants ────────────────────────────────────────────────────────────
ARM_LENGTH = 1.0

ALPHA_MIN, ALPHA_MAX = 0.0, 360.0
BETA_MIN,  BETA_MAX  = 0.0, 180.0
GAMMA_MIN, GAMMA_MAX = 0.0, 360.0

# ── Dark theme stylesheet ────────────────────────────────────────────────────
DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #1e1e2e;
    color: #cdd6f4;
    font-family: "Segoe UI", "Ubuntu", sans-serif;
    font-size: 13px;
}
QGroupBox {
    font-weight: bold;
    border: 1px solid #45475a;
    border-radius: 6px;
    margin-top: 10px;
    padding-top: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    color: #89b4fa;
}
QPushButton {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 5px;
    padding: 6px 14px;
    color: #cdd6f4;
    min-height: 22px;
}
QPushButton:hover { background-color: #45475a; }
QPushButton:pressed { background-color: #585b70; }
QPushButton:disabled { color: #585b70; border-color: #313244; }
QPushButton#estop {
    background-color: #d32f2f;
    color: white;
    font-size: 15px;
    font-weight: bold;
    border: 2px solid #f44336;
    min-height: 44px;
    border-radius: 6px;
}
QPushButton#estop:hover { background-color: #f44336; }
QPushButton#estop:pressed { background-color: #b71c1c; }
QPushButton#home_btn {
    background-color: #1e88e5;
    color: white;
    font-weight: bold;
    border: 1px solid #42a5f5;
    min-height: 32px;
}
QPushButton#home_btn:hover { background-color: #42a5f5; }
QDoubleSpinBox, QComboBox {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    padding: 3px 6px;
    color: #cdd6f4;
    min-height: 22px;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    width: 18px;
    background-color: #45475a;
    border: none;
}
QSlider::groove:horizontal {
    height: 6px;
    background: #45475a;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    width: 16px; height: 16px;
    margin: -5px 0;
    background: #89b4fa;
    border-radius: 8px;
}
QSlider::sub-page:horizontal { background: #89b4fa; border-radius: 3px; }
QTextEdit {
    background-color: #181825;
    border: 1px solid #45475a;
    border-radius: 4px;
    color: #a6adc8;
    font-family: "Cascadia Code", "Consolas", monospace;
    font-size: 11px;
}
QLabel#pos_label {
    font-family: "Cascadia Code", "Consolas", monospace;
    font-size: 16px;
    color: #a6e3a1;
    padding: 2px 0;
}
QLabel#angle_name { color: #89b4fa; font-weight: bold; }
QStatusBar { background-color: #181825; color: #a6adc8; border-top: 1px solid #313244; }
QStatusBar QLabel { color: #a6adc8; }
QProgressBar {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    text-align: center;
    color: #cdd6f4;
    max-height: 16px;
}
QProgressBar::chunk { background-color: #89b4fa; border-radius: 3px; }
QSplitter::handle { background-color: #45475a; width: 3px; }
"""


# ═════════════════════════════════════════════════════════════════════════════
#  Serial Interface
# ═════════════════════════════════════════════════════════════════════════════
class SerialInterface:
    """Thread-safe serial link to the Arduino controller."""

    def __init__(self):
        self.ser: serial.Serial | None = None
        self.connected = False
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @staticmethod
    def list_ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.connected = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self.rx_queue.put(line)
            except (serial.SerialException, OSError):
                self.connected = False
                break
            time.sleep(0.005)

    def _send(self, cmd: str) -> bool:
        if self.ser and self.ser.is_open:
            self.ser.write(f"{cmd}\n".encode())
            return True
        return False

    def send_move_abs(self, a, b, g, speed):
        return self._send(f"CMD:MOVE_ABS {a:.2f} {b:.2f} {g:.2f} {speed:.2f}")

    def send_move_rel(self, a, b, g, speed):
        return self._send(f"CMD:MOVE_REL {a:.2f} {b:.2f} {g:.2f} {speed:.2f}")

    def send_stop(self):
        return self._send("CMD:STOP")

    def send_home(self, speed: float = 30.0):
        return self._send(f"CMD:HOME {speed:.2f}")

    def send_query(self):
        return self._send("CMD:QUERY")

    def drain(self) -> list[str]:
        msgs: list[str] = []
        while not self.rx_queue.empty():
            try:
                msgs.append(self.rx_queue.get_nowait())
            except queue.Empty:
                break
        return msgs


# ═════════════════════════════════════════════════════════════════════════════
#  3-D Arm Visualisation (Matplotlib canvas for Qt)
# ═════════════════════════════════════════════════════════════════════════════
class ArmCanvas(FigureCanvasQTAgg):
    """Matplotlib 3-D viewport embedded as a QWidget."""

    def __init__(self, parent: QWidget | None = None):
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor="#1e1e2e")
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._style_axes()
        self.update_arm(0, 0, 0)

    # ── cosmetics ─────────────────────────────────────────────────────────
    def _style_axes(self):
        ax = self.ax
        ax.set_facecolor("#1e1e2e")
        lim = ARM_LENGTH * 1.25
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-0.15 * lim, 1.35 * lim)
        ax.set_xlabel("X", color="#f38ba8")
        ax.set_ylabel("Y", color="#a6e3a1")
        ax.set_zlabel("Z", color="#89b4fa")
        ax.tick_params(colors="#6c7086", labelsize=8)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor("#45475a")
        ax.yaxis.pane.set_edgecolor("#45475a")
        ax.zaxis.pane.set_edgecolor("#45475a")
        ax.view_init(elev=25, azim=45)

    @staticmethod
    def _cube_faces(center: np.ndarray, size: float, R: np.ndarray):
        s = size / 2
        pts = np.array([[-s,-s,-s],[s,-s,-s],[s,s,-s],[-s,s,-s],
                        [-s,-s, s],[s,-s, s],[s,s, s],[-s,s, s]])
        pts = (R @ pts.T).T + center
        idx = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[1,2,6,5],[0,3,7,4]]
        return [pts[f].tolist() for f in idx]

    # ── redraw ────────────────────────────────────────────────────────────
    def update_arm(self, alpha_deg: float, beta_deg: float, gamma_deg: float):
        self.ax.cla()
        self._style_axes()
        ax = self.ax
        a, b, g = np.radians(alpha_deg), np.radians(beta_deg), np.radians(gamma_deg)

        end = ARM_LENGTH * np.array([np.sin(b)*np.cos(a),
                                     np.sin(b)*np.sin(a),
                                     np.cos(b)])

        # base sphere
        u, v = np.meshgrid(np.linspace(0, 2*np.pi, 18),
                           np.linspace(0, np.pi, 12))
        r = 0.06
        ax.plot_surface(r*np.cos(u)*np.sin(v), r*np.sin(u)*np.sin(v),
                        r*np.cos(v), color="#585b70", alpha=0.9)

        # arm link
        ax.plot([0, end[0]], [0, end[1]], [0, end[2]],
                color="#89b4fa", linewidth=3.5, solid_capstyle="round")

        # origin axes
        L = 0.28
        for vec, c, lbl in [([L,0,0],"#f38ba8","X"),
                            ([0,L,0],"#a6e3a1","Y"),
                            ([0,0,L],"#89b4fa","Z")]:
            ax.quiver(0, 0, 0, *vec, color=c, arrow_length_ratio=0.15, linewidth=1.2)
            ax.text(*(np.array(vec)*1.15), lbl, color=c, fontsize=8)

        # α arc
        if abs(alpha_deg) > 0.5:
            t = np.linspace(0, a, 40)
            R_arc = 0.22
            ax.plot(R_arc*np.cos(t), R_arc*np.sin(t),
                    np.zeros_like(t), color="#f38ba8", lw=1.4)
        # β arc
        if abs(beta_deg) > 0.5:
            t = np.linspace(0, b, 40)
            R_arc = 0.28
            ax.plot(R_arc*np.sin(t)*np.cos(a), R_arc*np.sin(t)*np.sin(a),
                    R_arc*np.cos(t), color="#a6e3a1", lw=1.4)

        # end-effector cube
        arm_dir = end / (np.linalg.norm(end) + 1e-12)
        ref = np.array([0,0,1]) if abs(arm_dir[2]) < 0.95 else np.array([1,0,0])
        p1 = np.cross(arm_dir, ref); p1 /= np.linalg.norm(p1)
        p2 = np.cross(arm_dir, p1)
        cg, sg = np.cos(g), np.sin(g)
        rp1 =  cg*p1 + sg*p2
        rp2 = -sg*p1 + cg*p2
        R_ee = np.column_stack([rp1, rp2, arm_dir])

        faces = self._cube_faces(end, 0.14, R_ee)
        ax.add_collection3d(Poly3DCollection(
            faces, alpha=0.35, facecolor="#94e2d5", edgecolor="#1e1e2e", lw=1.5))

        # γ indicator tick
        tk_ = 0.10
        ax.plot([end[0], end[0]+tk_*rp1[0]],
                [end[1], end[1]+tk_*rp1[1]],
                [end[2], end[2]+tk_*rp1[2]],
                color="#f5c2e7", lw=2.2)

        ax.set_title(
            f"α = {alpha_deg:.1f}°    β = {beta_deg:.1f}°    γ = {gamma_deg:.1f}°",
            fontsize=11, pad=10, color="#cdd6f4")

        self.draw_idle()


# ═════════════════════════════════════════════════════════════════════════════
#  Thread-safe signal bridge (seq worker → GUI)
# ═════════════════════════════════════════════════════════════════════════════
class _Signals(QObject):
    log       = pyqtSignal(str)
    set_pos   = pyqtSignal(float, float, float)
    seq_label = pyqtSignal(str)
    seq_done  = pyqtSignal()


# ═════════════════════════════════════════════════════════════════════════════
#  Main Window
# ═════════════════════════════════════════════════════════════════════════════
class RobotControlWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Controller")
        self.resize(1340, 860)
        self.setMinimumSize(1060, 720)

        self.serial = SerialInterface()

        # tracked state
        self.cur_a = 0.0
        self.cur_b = 0.0
        self.cur_g = 0.0

        self.seq_data: list[dict] | None = None
        self._seq_stop = threading.Event()
        self._seq_running = False
        self._signals = _Signals()
        self._signals.log.connect(self._append_log)
        self._signals.set_pos.connect(self._sim_set)
        self._signals.seq_label.connect(lambda t: self.seq_lbl.setText(t))
        self._signals.seq_done.connect(self._on_seq_done)

        self._build_ui()

        # serial poll timer
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_serial)
        self._poll_timer.start(50)

    # ─────────────────────────────────────────────────────────────────────
    #  UI construction
    # ─────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(6, 6, 6, 0)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root_layout.addWidget(splitter, stretch=1)

        # ── LEFT: 3-D viewport ────────────────────────────────────────────
        plot_container = QWidget()
        plot_vbox = QVBoxLayout(plot_container)
        plot_vbox.setContentsMargins(0, 0, 0, 0)
        self.canvas = ArmCanvas()
        self.toolbar = NavigationToolbar2QT(self.canvas, plot_container)
        self.toolbar.setStyleSheet("background: #181825; border: none;")
        plot_vbox.addWidget(self.toolbar)
        plot_vbox.addWidget(self.canvas, stretch=1)
        splitter.addWidget(plot_container)

        # ── RIGHT: controls ───────────────────────────────────────────────
        ctrl = QWidget()
        ctrl.setMaximumWidth(420)
        ctrl.setMinimumWidth(300)
        ctrl_layout = QVBoxLayout(ctrl)
        ctrl_layout.setContentsMargins(4, 0, 4, 4)
        ctrl_layout.setSpacing(6)
        splitter.addWidget(ctrl)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        # ── Connection ────────────────────────────────────────────────────
        conn_grp = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_grp)
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Port:"))
        self.port_cb = QComboBox()
        self.port_cb.setMinimumWidth(140)
        row1.addWidget(self.port_cb, stretch=1)
        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setFixedWidth(36)
        self.refresh_btn.clicked.connect(self._refresh_ports)
        row1.addWidget(self.refresh_btn)
        conn_layout.addLayout(row1)

        row2 = QHBoxLayout()
        self.conn_btn = QPushButton("Connect")
        self.conn_btn.clicked.connect(self._toggle_conn)
        row2.addWidget(self.conn_btn)
        self.conn_indicator = QLabel("● Disconnected")
        self.conn_indicator.setStyleSheet("color: #f38ba8; font-weight: bold;")
        row2.addWidget(self.conn_indicator)
        row2.addStretch()
        conn_layout.addLayout(row2)
        ctrl_layout.addWidget(conn_grp)
        self._refresh_ports()

        # ── Manual Control ────────────────────────────────────────────────
        man_grp = QGroupBox("Manual Control")
        man_layout = QVBoxLayout(man_grp)

        self.spin_a, self.slider_a = self._angle_row(man_layout, "α  XY azimuth",     ALPHA_MIN, ALPHA_MAX)
        self.spin_b, self.slider_b = self._angle_row(man_layout, "β  Z elevation",     BETA_MIN,  BETA_MAX)
        self.spin_g, self.slider_g = self._angle_row(man_layout, "γ  End-eff. roll",   GAMMA_MIN, GAMMA_MAX)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed (°/s):"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(1, 360)
        self.speed_spin.setValue(30)
        self.speed_spin.setDecimals(1)
        speed_row.addWidget(self.speed_spin)
        speed_row.addStretch()
        man_layout.addLayout(speed_row)

        btn_row = QHBoxLayout()
        for text, slot in [("Send Absolute", self._cmd_abs),
                           ("Send Relative", self._cmd_rel),
                           ("Preview",       self._cmd_preview)]:
            b = QPushButton(text)
            b.clicked.connect(slot)
            btn_row.addWidget(b)
        man_layout.addLayout(btn_row)
        ctrl_layout.addWidget(man_grp)

        # ── Actual Position ───────────────────────────────────────────────
        pos_grp = QGroupBox("Actual Position")
        pos_layout = QGridLayout(pos_grp)
        self.pos_labels: dict[str, QLabel] = {}
        for row, (sym, name) in enumerate([("α", "XY azimuth"),
                                            ("β", "Z elevation"),
                                            ("γ", "Roll")]):
            lbl_name = QLabel(f"{sym}  {name}")
            lbl_name.setObjectName("angle_name")
            pos_layout.addWidget(lbl_name, row, 0)
            lbl_val = QLabel("0.00°")
            lbl_val.setObjectName("pos_label")
            lbl_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            pos_layout.addWidget(lbl_val, row, 1)
            self.pos_labels[sym] = lbl_val
        ctrl_layout.addWidget(pos_grp)

        # ── Command Sequence ──────────────────────────────────────────────
        seq_grp = QGroupBox("Command Sequence")
        seq_layout = QVBoxLayout(seq_grp)
        load_btn = QPushButton("Load JSON…")
        load_btn.clicked.connect(self._load_seq)
        seq_layout.addWidget(load_btn)
        self.seq_lbl = QLabel("No sequence loaded")
        seq_layout.addWidget(self.seq_lbl)
        self.seq_progress = QProgressBar()
        self.seq_progress.setVisible(False)
        seq_layout.addWidget(self.seq_progress)
        seq_btn_row = QHBoxLayout()
        self.run_btn = QPushButton("▶  Run")
        self.run_btn.setEnabled(False)
        self.run_btn.clicked.connect(self._run_seq)
        seq_btn_row.addWidget(self.run_btn)
        self.stop_btn = QPushButton("■  Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._stop_seq)
        seq_btn_row.addWidget(self.stop_btn)
        seq_layout.addLayout(seq_btn_row)
        ctrl_layout.addWidget(seq_grp)

        # ── E-stop / Home ─────────────────────────────────────────────────
        estop = QPushButton("⚠  EMERGENCY STOP")
        estop.setObjectName("estop")
        estop.clicked.connect(self._estop)
        ctrl_layout.addWidget(estop)

        home = QPushButton("⌂  Home (Zero All)")
        home.setObjectName("home_btn")
        home.clicked.connect(self._home)
        ctrl_layout.addWidget(home)

        # ── Log ───────────────────────────────────────────────────────────
        log_grp = QGroupBox("Log")
        log_layout = QVBoxLayout(log_grp)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        log_layout.addWidget(self.log_box)
        ctrl_layout.addWidget(log_grp, stretch=1)

        # ── Status bar ────────────────────────────────────────────────────
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.mode_label = QLabel("  Mode: SIMULATION  ")
        self.mode_label.setStyleSheet(
            "background: #fab387; color: #1e1e2e; font-weight: bold; "
            "padding: 2px 8px; border-radius: 3px;")
        self.status.addPermanentWidget(self.mode_label)
        self.status.showMessage("Ready — no Arduino connected (simulation mode)")

        # ── initial render ────────────────────────────────────────────────
        self.canvas.update_arm(0, 0, 0)

    # ── angle row helper ──────────────────────────────────────────────────
    def _angle_row(self, layout: QVBoxLayout, label: str,
                   lo: float, hi: float) -> tuple[QDoubleSpinBox, QSlider]:
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setFixedWidth(120)
        row.addWidget(lbl)

        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(1)
        spin.setSingleStep(1.0)
        spin.setFixedWidth(80)
        row.addWidget(spin)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(int(lo * 10), int(hi * 10))
        slider.setValue(0)
        row.addWidget(slider, stretch=1)

        # bidirectional sync
        spin.valueChanged.connect(lambda v: slider.setValue(int(v * 10)))
        slider.valueChanged.connect(lambda v: spin.setValue(v / 10.0))

        layout.addLayout(row)
        return spin, slider

    # ── port helpers ──────────────────────────────────────────────────────
    def _refresh_ports(self):
        self.port_cb.clear()
        ports = SerialInterface.list_ports()
        self.port_cb.addItems(ports)

    # ── logging ───────────────────────────────────────────────────────────
    def _append_log(self, msg: str):
        self.log_box.append(f"<span style='color:#6c7086'>[{time.strftime('%H:%M:%S')}]</span> {msg}")

    def _update_pos_labels(self):
        self.pos_labels["α"].setText(f"{self.cur_a:>8.2f}°")
        self.pos_labels["β"].setText(f"{self.cur_b:>8.2f}°")
        self.pos_labels["γ"].setText(f"{self.cur_g:>8.2f}°")

    # ── connection ────────────────────────────────────────────────────────
    def _toggle_conn(self):
        if self.serial.connected:
            self.serial.disconnect()
            self.conn_btn.setText("Connect")
            self.conn_indicator.setText("● Disconnected")
            self.conn_indicator.setStyleSheet("color: #f38ba8; font-weight: bold;")
            self.mode_label.setText("  Mode: SIMULATION  ")
            self.mode_label.setStyleSheet(
                "background: #fab387; color: #1e1e2e; font-weight: bold; "
                "padding: 2px 8px; border-radius: 3px;")
            self.status.showMessage("Disconnected — simulation mode")
            self._append_log("Disconnected")
        else:
            port = self.port_cb.currentText()
            if not port:
                QMessageBox.warning(self, "No Port", "Select a serial port first.")
                return
            try:
                self.serial.connect(port)
                self.conn_btn.setText("Disconnect")
                self.conn_indicator.setText("● Connected")
                self.conn_indicator.setStyleSheet("color: #a6e3a1; font-weight: bold;")
                self.mode_label.setText("  Mode: LIVE  ")
                self.mode_label.setStyleSheet(
                    "background: #a6e3a1; color: #1e1e2e; font-weight: bold; "
                    "padding: 2px 8px; border-radius: 3px;")
                self.status.showMessage(f"Connected to {port} @ 115200")
                self._append_log(f"Connected to {port} @ 115200")
            except serial.SerialException as e:
                QMessageBox.critical(self, "Connection Error", str(e))
                self._append_log(f"Connection failed: {e}")

    # ── manual commands ───────────────────────────────────────────────────
    @staticmethod
    def _clamp(a, b, g):
        return (float(np.clip(a, ALPHA_MIN, ALPHA_MAX)),
                float(np.clip(b, BETA_MIN,  BETA_MAX)),
                float(np.clip(g, GAMMA_MIN, GAMMA_MAX)))

    def _read_spins(self):
        return self.spin_a.value(), self.spin_b.value(), self.spin_g.value()

    def _cmd_abs(self):
        a, b, g = self._clamp(*self._read_spins())
        spd = self.speed_spin.value()
        if self.serial.connected:
            self.serial.send_move_abs(a, b, g, spd)
            self._append_log(f"TX MOVE_ABS  α={a:.1f}  β={b:.1f}  γ={g:.1f}  spd={spd:.0f}")
        else:
            self._sim_set(a, b, g)
            self._append_log(f"<b>[SIM]</b> ABS  α={a:.1f}  β={b:.1f}  γ={g:.1f}")

    def _cmd_rel(self):
        da, db, dg = self._read_spins()
        spd = self.speed_spin.value()
        if self.serial.connected:
            self.serial.send_move_rel(da, db, dg, spd)
            self._append_log(f"TX MOVE_REL  Δα={da:.1f}  Δβ={db:.1f}  Δγ={dg:.1f}  spd={spd:.0f}")
        else:
            a, b, g = self._clamp(self.cur_a + da, self.cur_b + db, self.cur_g + dg)
            self._sim_set(a, b, g)
            self._append_log(f"<b>[SIM]</b> REL → α={a:.1f}  β={b:.1f}  γ={g:.1f}")

    def _cmd_preview(self):
        a, b, g = self._clamp(*self._read_spins())
        self.canvas.update_arm(a, b, g)
        self._append_log(f"Preview  α={a:.1f}  β={b:.1f}  γ={g:.1f}")

    def _sim_set(self, a: float, b: float, g: float):
        self.cur_a, self.cur_b, self.cur_g = a, b, g
        self._update_pos_labels()
        self.canvas.update_arm(a, b, g)

    # ── E-stop / Home ─────────────────────────────────────────────────────
    def _estop(self):
        self._seq_stop.set()
        if self.serial.connected:
            self.serial.send_stop()
        self._append_log("<span style='color:#f38ba8;font-weight:bold'>⚠ EMERGENCY STOP</span>")

    def _home(self):
        spd = self.speed_spin.value()
        if self.serial.connected:
            self.serial.send_home(spd)
            self._append_log(f"TX HOME  spd={spd:.0f}")
        else:
            self._sim_set(0, 0, 0)
            self.spin_a.setValue(0); self.spin_b.setValue(0); self.spin_g.setValue(0)
            self._append_log("<b>[SIM]</b> HOME → 0, 0, 0")

    # ── JSON sequence ─────────────────────────────────────────────────────
    def _load_seq(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Command Sequence", "",
            "JSON files (*.json);;All files (*)")
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            if not isinstance(data, list):
                raise ValueError("Root must be a JSON array of commands")
            valid = {"move_abs", "move_rel", "delay", "home"}
            for i, c in enumerate(data):
                if "command" not in c:
                    raise ValueError(f"Step {i+1}: missing 'command' key")
                if c["command"] not in valid:
                    raise ValueError(f"Step {i+1}: unknown command '{c['command']}'")
            self.seq_data = data
            name = os.path.basename(path)
            self.seq_lbl.setText(f"{len(data)} steps — {name}")
            self.run_btn.setEnabled(True)
            self._append_log(f"Loaded {len(data)} steps from {name}")
        except (json.JSONDecodeError, ValueError, OSError) as e:
            QMessageBox.critical(self, "Sequence Error", str(e))
            self._append_log(f"Load error: {e}")

    def _run_seq(self):
        if not self.seq_data or self._seq_running:
            return
        self._seq_stop.clear()
        self._seq_running = True
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.seq_progress.setRange(0, len(self.seq_data))
        self.seq_progress.setValue(0)
        self.seq_progress.setVisible(True)
        threading.Thread(target=self._seq_worker, daemon=True).start()

    def _stop_seq(self):
        self._seq_stop.set()
        self._estop()

    def _on_seq_done(self):
        self._seq_running = False
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.seq_progress.setVisible(False)

    def _seq_worker(self):
        sig = self._signals
        try:
            for i, step in enumerate(self.seq_data):
                if self._seq_stop.is_set():
                    sig.log.emit("Sequence aborted")
                    break

                cmd = step["command"]
                sig.seq_label.emit(f"Running {i+1}/{len(self.seq_data)}  ({cmd})")

                if cmd == "move_abs":
                    a = step.get("alpha", self.cur_a)
                    b = step.get("beta",  self.cur_b)
                    g = step.get("gamma", self.cur_g)
                    spd = step.get("speed", 30.0)
                    a, b, g = self._clamp(a, b, g)
                    if self.serial.connected:
                        self.serial.send_move_abs(a, b, g, spd)
                        self._wait_done()
                    else:
                        sig.set_pos.emit(a, b, g)
                        time.sleep(0.3)
                    sig.log.emit(f"SEQ ABS  α={a:.1f} β={b:.1f} γ={g:.1f} spd={spd:.0f}")

                elif cmd == "move_rel":
                    da = step.get("alpha", 0.0)
                    db = step.get("beta",  0.0)
                    dg = step.get("gamma", 0.0)
                    spd = step.get("speed", 30.0)
                    if self.serial.connected:
                        self.serial.send_move_rel(da, db, dg, spd)
                        self._wait_done()
                    else:
                        a, b, g = self._clamp(self.cur_a + da, self.cur_b + db,
                                              self.cur_g + dg)
                        sig.set_pos.emit(a, b, g)
                        time.sleep(0.3)
                    sig.log.emit(f"SEQ REL  Δα={da:.1f} Δβ={db:.1f} Δγ={dg:.1f} spd={spd:.0f}")

                elif cmd == "delay":
                    t = step.get("time", 1.0)
                    sig.log.emit(f"SEQ DELAY {t:.2f}s")
                    deadline = time.time() + t
                    while time.time() < deadline and not self._seq_stop.is_set():
                        time.sleep(0.05)

                elif cmd == "home":
                    spd = step.get("speed", 30.0)
                    if self.serial.connected:
                        self.serial.send_home(spd)
                        self._wait_done()
                    else:
                        sig.set_pos.emit(0.0, 0.0, 0.0)
                        time.sleep(0.3)
                    sig.log.emit(f"SEQ HOME  spd={spd:.0f}")

                # update progress bar in main thread is safe via signal
                self.seq_progress.setValue(i + 1)   # works cross-thread for QProgressBar
            else:
                sig.log.emit("<b>Sequence complete ✓</b>")
                sig.seq_label.emit("Sequence complete")
        except Exception as exc:
            sig.log.emit(f"Sequence error: {exc}")
        finally:
            sig.seq_done.emit()

    def _wait_done(self, timeout: float = 60.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and not self._seq_stop.is_set():
            for msg in self.serial.drain():
                self._signals.log.emit(f"RX {msg}")
                if msg.strip() == "DONE":
                    return True
            time.sleep(0.05)
        return False

    # ── serial polling ────────────────────────────────────────────────────
    def _poll_serial(self):
        if not self.serial.connected:
            return
        for msg in self.serial.drain():
            self._handle_rx(msg)

    def _handle_rx(self, msg: str):
        if msg.startswith("POS:"):
            try:
                parts = msg[4:].split(",")
                self.cur_a = float(parts[0])
                self.cur_b = float(parts[1])
                self.cur_g = float(parts[2])
                self._update_pos_labels()
                self.canvas.update_arm(self.cur_a, self.cur_b, self.cur_g)
            except (ValueError, IndexError):
                self._append_log(f"Bad POS msg: {msg}")
        elif msg.startswith("ACK:"):
            self._append_log(f"RX {msg}")
        elif msg.startswith("ERR:"):
            self._append_log(f"<span style='color:#f38ba8'>⚠ {msg}</span>")
        elif msg.strip() == "DONE":
            self._append_log("RX DONE")
        else:
            self._append_log(f"RX? {msg}")

    # ── cleanup ───────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self._seq_stop.set()
        self.serial.disconnect()
        event.accept()


# ═════════════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_STYLE)
    win = RobotControlWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()