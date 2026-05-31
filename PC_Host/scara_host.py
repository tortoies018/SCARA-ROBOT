import sys
import math
from typing import Optional

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QComboBox, QLabel, QTextEdit, QLineEdit,
    QSplitter, QFormLayout, QDoubleSpinBox, QSpinBox, QSlider,
    QMessageBox, QStatusBar, QGridLayout, QCheckBox
)
from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QMouseEvent, QPaintEvent
from PyQt6.QtCore import QSize

from scara_serial import SCARASerial
from scara_kinematics import (
    forward_kinematics, inverse_kinematics, cartesian_to_steps,
    steps_to_cartesian, interpolate_line, interpolate_arc,
    dda_interpolate_segments, ARM1_LENGTH, ARM2_LENGTH
)


class SCARAWidget(QWidget):
    mouse_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 500)
        self.setMouseTracking(True)

        self.theta1: float = 0.0
        self.theta2: float = 0.0
        self.ee_x: float = ARM1_LENGTH + ARM2_LENGTH
        self.ee_y: float = 0.0

        self.traj_points: list[tuple[float, float]] = []
        self.temp_points: list[tuple[float, float]] = []
        self.home_pos: tuple[float, float] = (ARM1_LENGTH + ARM2_LENGTH, 0.0)
        self.origin: QPointF = QPointF()

        self.scale: float = 1.5
        self.mouse_pos: Optional[QPointF] = None

    def set_angles(self, t1: float, t2: float):
        self.theta1 = t1
        self.theta2 = t2
        import math
        r1 = math.radians(t1)
        r2 = math.radians(t2)
        self.ee_x = ARM1_LENGTH * math.cos(r1) + ARM2_LENGTH * math.cos(r1 + r2)
        self.ee_y = ARM1_LENGTH * math.sin(r1) + ARM2_LENGTH * math.sin(r1 + r2)
        self.update()

    def set_trajectory(self, pts: list[tuple[float, float]]):
        self.traj_points = list(pts)
        self.update()

    def clear_trajectory(self):
        self.traj_points.clear()
        self.temp_points.clear()
        self.update()

    def paintEvent(self, event: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        cx, cy = w / 2.0, h - 60

        self.origin = QPointF(cx, cy)

        p.setPen(QPen(QColor(200, 200, 200), 1, Qt.PenStyle.DotLine))
        grid_size = 25
        for x in range(0, w, grid_size):
            p.drawLine(int(x), 0, int(x), h)
        for y in range(0, h, grid_size):
            p.drawLine(0, int(y), w, int(y))

        p.setPen(Qt.GlobalColor.gray)
        p.drawLine(0, int(cy), w, int(cy))
        p.drawLine(int(cx), 0, int(cx), h)

        font = QFont("Monospace", 8)
        p.setFont(font)
        for x in range(0, w, 50):
            p.drawText(int(x) - 10, int(cy) + 15, str(x - int(cx)))
        for y in range(0, h, 50):
            p.drawText(int(cx) + 5, int(y) + 3, str(int(cy) - y))

        p.translate(cx, cy)
        p.scale(1, -1)

        scale = self.scale
        p.scale(scale, scale)

        p.setPen(QPen(QColor(100, 100, 255, 100), 2.0 / scale))
        p.setBrush(QBrush(QColor(100, 100, 255, 30)))
        for pt in self.traj_points:
            px, py = pt
            p.drawEllipse(QPointF(px, py), 2.0 / scale, 2.0 / scale)

        if len(self.traj_points) > 1:
            p.setPen(QPen(QColor(100, 100, 255), 1.5 / scale))
            for i in range(len(self.traj_points) - 1):
                x1, y1 = self.traj_points[i]
                x2, y2 = self.traj_points[i + 1]
                p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        if self.temp_points:
            p.setPen(QPen(QColor(255, 100, 100), 1.5 / scale, Qt.PenStyle.DashLine))
            for i in range(len(self.temp_points) - 1):
                x1, y1 = self.temp_points[i]
                x2, y2 = self.temp_points[i + 1]
                p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
            for pt in self.temp_points:
                px, py = pt
                p.drawEllipse(QPointF(px, py), 1.5 / scale, 1.5 / scale)

        p.setPen(QPen(Qt.GlobalColor.darkGreen, 3.0 / scale))
        p.drawEllipse(QPointF(self.home_pos[0], self.home_pos[1]), 3.0 / scale, 3.0 / scale)
        p.setPen(Qt.GlobalColor.black)
        p.drawText(QPointF(self.home_pos[0] - 8, self.home_pos[1] - 10), "HOME")

        p.setPen(QPen(Qt.GlobalColor.black, 2.0 / scale))
        import math as m
        r1 = m.radians(self.theta1)
        r2 = m.radians(self.theta2)
        j1_x = ARM1_LENGTH * m.cos(r1)
        j1_y = ARM1_LENGTH * m.sin(r1)
        ee_x_f = ARM1_LENGTH * m.cos(r1) + ARM2_LENGTH * m.cos(r1 + r2)
        ee_y_f = ARM1_LENGTH * m.sin(r1) + ARM2_LENGTH * m.sin(r1 + r2)

        p.setPen(QPen(QColor(200, 50, 50), 4.0 / scale))
        p.drawLine(QPointF(0, 0), QPointF(j1_x, j1_y))
        p.setPen(QPen(QColor(50, 50, 200), 4.0 / scale))
        p.drawLine(QPointF(j1_x, j1_y), QPointF(ee_x_f, ee_y_f))

        p.setPen(Qt.GlobalColor.black)
        p.setBrush(QBrush(Qt.GlobalColor.red))
        p.drawEllipse(QPointF(0, 0), 4.0 / scale, 4.0 / scale)

        p.setBrush(QBrush(Qt.GlobalColor.green))
        p.drawEllipse(QPointF(j1_x, j1_y), 3.0 / scale, 3.0 / scale)

        p.setBrush(QBrush(Qt.GlobalColor.blue))
        p.drawEllipse(QPointF(ee_x_f, ee_y_f), 3.0 / scale, 3.0 / scale)

        p.setPen(Qt.GlobalColor.black)
        p.drawText(QPointF(0, -15), "Base")
        p.drawText(QPointF(j1_x - 5, j1_y - 15), "J1")
        p.drawText(QPointF(ee_x_f - 5, ee_y_f - 15),
                   f"({ee_x_f:.1f},{ee_y_f:.1f})")

        if self.mouse_pos:
            mp = self.mouse_pos
            p.setPen(QPen(QColor(0, 0, 0, 100), 1.0 / scale, Qt.PenStyle.DashLine))
            p.drawLine(QPointF(mp.x(), -1000), QPointF(mp.x(), 1000))
            p.drawLine(QPointF(-1000, mp.y()), QPointF(1000, mp.y()))

        p.resetTransform()
        p.setPen(Qt.GlobalColor.black)
        p.drawText(10, 20,
                   f"θ1={self.theta1:.1f}°  θ2={self.theta2:.1f}°  "
                   f"EE=({ee_x_f:.1f}, {ee_y_f:.1f})")

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            wpos = self._widget_to_scene(event.position())
            self.mouse_clicked.emit(wpos.x(), wpos.y())

    def mouseMoveEvent(self, event: QMouseEvent):
        wpos = self._widget_to_scene(event.position())
        self.mouse_pos = QPointF(wpos.x(), wpos.y())
        self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.scale *= 1.1
        else:
            self.scale /= 1.1
        self.scale = max(0.3, min(10.0, self.scale))
        self.update()

    def _widget_to_scene(self, pos: QPointF) -> QPointF:
        cx = self.width() / 2.0
        cy = self.height() - 60
        wx = (pos.x() - cx) / self.scale
        wy = (cy - pos.y()) / self.scale
        return QPointF(wx, wy)

    def resizeEvent(self, event):
        self.update()
        super().resizeEvent(event)


class SCARAHost(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial = SCARASerial()
        self.traj_points: list[tuple[float, float]] = []
        self.drawing_mode = "line"
        self.arc_center: Optional[tuple[float, float]] = None
        self.arc_ccw = True
        self.current_segments: list[dict] = []
        self._build_ui()
        self._setup_timers()

        self.setWindowTitle("SCARA Robot Control")
        self.resize(1200, 800)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(8)

        conn_group = QGroupBox("Connection")
        conn_layout = QGridLayout(conn_group)
        conn_layout.addWidget(QLabel("Port:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(120)
        conn_layout.addWidget(self.port_combo, 0, 1)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(self.refresh_btn, 0, 2)
        conn_layout.addWidget(QLabel("Baud:"), 1, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baud_combo.setCurrentText("115200")
        conn_layout.addWidget(self.baud_combo, 1, 1)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        conn_layout.addWidget(self.connect_btn, 1, 2)
        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet("color: red")
        conn_layout.addWidget(self.conn_status, 2, 0, 1, 3)
        left_layout.addWidget(conn_group)

        ctrl_group = QGroupBox("Robot Control")
        ctrl_layout = QGridLayout(ctrl_group)
        self.home_btn = QPushButton("Home")
        self.home_btn.clicked.connect(self._cmd_home)
        ctrl_layout.addWidget(self.home_btn, 0, 0)
        self.pen_up_btn = QPushButton("Pen Up")
        self.pen_up_btn.clicked.connect(lambda: self._send_cmd("P0\r\n"))
        ctrl_layout.addWidget(self.pen_up_btn, 0, 1)
        self.pen_down_btn = QPushButton("Pen Down")
        self.pen_down_btn.clicked.connect(lambda: self._send_cmd("P1\r\n"))
        ctrl_layout.addWidget(self.pen_down_btn, 0, 2)
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet("background-color: #ff4444; color: white")
        self.stop_btn.clicked.connect(self._cmd_stop)
        ctrl_layout.addWidget(self.stop_btn, 1, 0, 1, 3)
        self.query_btn = QPushButton("Query")
        self.query_btn.clicked.connect(self._cmd_query)
        ctrl_layout.addWidget(self.query_btn, 2, 0, 1, 3)
        left_layout.addWidget(ctrl_group)

        fk_group = QGroupBox("Forward Kinematics")
        fk_layout = QFormLayout(fk_group)
        self.fk_t1 = QDoubleSpinBox()
        self.fk_t1.setRange(-180, 180)
        self.fk_t1.setSuffix("°")
        fk_layout.addRow("θ1:", self.fk_t1)
        self.fk_t2 = QDoubleSpinBox()
        self.fk_t2.setRange(-180, 180)
        self.fk_t2.setSuffix("°")
        fk_layout.addRow("θ2:", self.fk_t2)
        self.fk_btn = QPushButton("Compute FK")
        self.fk_btn.clicked.connect(self._compute_fk)
        fk_layout.addRow(self.fk_btn)
        self.fk_result = QLabel("x: 0.0  y: 0.0")
        fk_layout.addRow(self.fk_result)
        self.fk_send_btn = QPushButton("Move to Position")
        self.fk_send_btn.clicked.connect(self._fk_send)
        fk_layout.addRow(self.fk_send_btn)
        left_layout.addWidget(fk_group)

        ik_group = QGroupBox("Inverse Kinematics")
        ik_layout = QFormLayout(ik_group)
        self.ik_x = QDoubleSpinBox()
        self.ik_x.setRange(-300, 300)
        ik_layout.addRow("X (mm):", self.ik_x)
        self.ik_y = QDoubleSpinBox()
        self.ik_y.setRange(-300, 300)
        ik_layout.addRow("Y (mm):", self.ik_y)
        self.ik_btn = QPushButton("Compute IK")
        self.ik_btn.clicked.connect(self._compute_ik)
        ik_layout.addRow(self.ik_btn)
        self.ik_result = QLabel("θ1: 0.0°  θ2: 0.0°")
        ik_layout.addRow(self.ik_result)
        self.ik_send_btn = QPushButton("Move to Position")
        self.ik_send_btn.clicked.connect(self._ik_send)
        ik_layout.addRow(self.ik_send_btn)
        left_layout.addWidget(ik_group)

        traj_group = QGroupBox("Trajectory")
        traj_layout = QVBoxLayout(traj_group)

        mode_row = QHBoxLayout()
        self.line_btn = QPushButton("Line")
        self.line_btn.setCheckable(True)
        self.line_btn.setChecked(True)
        self.line_btn.clicked.connect(lambda: self._set_draw_mode("line"))
        self.arc_btn = QPushButton("Arc")
        self.arc_btn.setCheckable(True)
        self.arc_btn.clicked.connect(lambda: self._set_draw_mode("arc"))
        mode_row.addWidget(self.line_btn)
        mode_row.addWidget(self.arc_btn)
        self.ccw_cb = QCheckBox("CCW")
        self.ccw_cb.setChecked(True)
        mode_row.addWidget(self.ccw_cb)
        traj_layout.addLayout(mode_row)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(100, 20000)
        self.speed_slider.setValue(2000)
        speed_row.addWidget(self.speed_slider)
        self.speed_label = QLabel("2000")
        speed_row.addWidget(self.speed_label)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(str(v)))
        traj_layout.addLayout(speed_row)

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step (mm):"))
        self.step_mm = QDoubleSpinBox()
        self.step_mm.setRange(0.1, 10.0)
        self.step_mm.setValue(1.0)
        self.step_mm.setSingleStep(0.1)
        step_row.addWidget(self.step_mm)
        traj_layout.addLayout(step_row)

        btn_row = QHBoxLayout()
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self._clear_traj)
        btn_row.addWidget(self.clear_btn)
        self.preview_btn = QPushButton("Preview")
        self.preview_btn.clicked.connect(self._preview_traj)
        btn_row.addWidget(self.preview_btn)
        traj_layout.addLayout(btn_row)

        self.send_traj_btn = QPushButton("Send Trajectory to Robot")
        self.send_traj_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold")
        self.send_traj_btn.clicked.connect(self._send_trajectory)
        traj_layout.addWidget(self.send_traj_btn)

        self.progress_label = QLabel("")
        traj_layout.addWidget(self.progress_label)
        left_layout.addWidget(traj_group)

        left_layout.addStretch()
        splitter.addWidget(left_panel)

        self.canvas = SCARAWidget()
        self.canvas.mouse_clicked.connect(self._canvas_click)
        splitter.addWidget(self.canvas)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        console_group = QGroupBox("Console")
        console_layout = QVBoxLayout(console_group)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setFont(QFont("Consolas", 10))
        self.console.setMaximumHeight(200)
        console_layout.addWidget(self.console)

        cmd_row = QHBoxLayout()
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Enter command...")
        self.cmd_input.returnPressed.connect(self._send_raw_cmd)
        cmd_row.addWidget(self.cmd_input)
        self.cmd_send_btn = QPushButton("Send")
        self.cmd_send_btn.clicked.connect(self._send_raw_cmd)
        cmd_row.addWidget(self.cmd_send_btn)
        console_layout.addLayout(cmd_row)
        right_layout.addWidget(console_group)
        right_layout.addStretch()
        splitter.addWidget(right_panel)

        splitter.setSizes([300, 600, 200])
        main_layout.addWidget(splitter)

        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Ready")

    def _setup_timers(self):
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self._poll_serial)
        self._poll_timer.start(200)

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in self.serial.list_ports():
            self.port_combo.addItem(p)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found")

    def _toggle_connect(self):
        if self.serial.connected:
            self.serial.disconnect()
            self.connect_btn.setText("Connect")
            self.conn_status.setText("Disconnected")
            self.conn_status.setStyleSheet("color: red")
        else:
            port = self.port_combo.currentText()
            if not port or port == "No ports found":
                return
            baud = int(self.baud_combo.currentText())
            if self.serial.connect(port, baud):
                self.connect_btn.setText("Disconnect")
                self.conn_status.setText("Connected")
                self.conn_status.setStyleSheet("color: green")
                self._log("Connected to " + port)
            else:
                QMessageBox.warning(self, "Error", f"Failed to connect to {port}")

    def _log(self, msg: str):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.console.append(f"[{ts}] {msg}")

    def _send_raw_cmd(self):
        txt = self.cmd_input.text().strip()
        if not txt:
            return
        self.cmd_input.clear()
        self._log(f">>> {txt}")
        resp = self.serial.send(txt + "\r\n")
        self._log(f"<<< {resp}")

    def _send_cmd(self, cmd: str):
        self._log(f">>> {cmd.strip()}")
        resp = self.serial.send(cmd)
        self._log(f"<<< {resp}")

    def _cmd_home(self):
        self._send_cmd("H\r\n")

    def _cmd_stop(self):
        self._send_cmd("!\r\n")

    def _cmd_query(self):
        resp = self.serial.query()
        self._log(f">>> Q")
        self._log(f"<<< {resp}")
        parsed = self.serial.parse_position(resp)
        if parsed:
            s1, s2, status = parsed
            x, y = steps_to_cartesian(s1, s2)
            self._log(f"  Position: ({x:.1f}, {y:.1f}) mm  Status: {status}")

    def _compute_fk(self):
        t1 = self.fk_t1.value()
        t2 = self.fk_t2.value()
        x, y = forward_kinematics(t1, t2)
        self.fk_result.setText(f"x: {x:.2f}  y: {y:.2f}")
        self.canvas.set_angles(t1, t2)
        self._log(f"FK: θ1={t1:.1f}° θ2={t2:.1f}° → ({x:.2f}, {y:.2f})")

    def _fk_send(self):
        t1 = self.fk_t1.value()
        t2 = self.fk_t2.value()
        from scara_kinematics import angle_to_steps
        s1 = angle_to_steps(t1)
        s2 = angle_to_steps(t2)
        resp = self.serial.move_absolute(s1, s2, self.speed_slider.value())
        self._log(f">>> A {s1} {s2} {self.speed_slider.value()}")
        self._log(f"<<< {resp}")

    def _compute_ik(self):
        x = self.ik_x.value()
        y = self.ik_y.value()
        ik = inverse_kinematics(x, y)
        if ik is None:
            self.ik_result.setText("UNREACHABLE")
            self._log("IK: Position unreachable")
            return
        t1, t2 = ik
        self.ik_result.setText(f"θ1: {t1:.2f}°  θ2: {t2:.2f}°")
        self.canvas.set_angles(t1, t2)
        self._log(f"IK: ({x:.2f}, {y:.2f}) → θ1={t1:.2f}° θ2={t2:.2f}°")

    def _ik_send(self):
        x = self.ik_x.value()
        y = self.ik_y.value()
        ik = inverse_kinematics(x, y)
        if ik is None:
            self._log("IK: Position unreachable")
            return
        t1, t2 = ik
        from scara_kinematics import angle_to_steps
        s1 = angle_to_steps(t1)
        s2 = angle_to_steps(t2)
        resp = self.serial.move_absolute(s1, s2, self.speed_slider.value())
        self._log(f">>> A {s1} {s2} {self.speed_slider.value()}")
        self._log(f"<<< {resp}")

    def _set_draw_mode(self, mode: str):
        self.drawing_mode = mode
        self.line_btn.setChecked(mode == "line")
        self.arc_btn.setChecked(mode == "arc")
        self.arc_center = None
        self.canvas.clear_trajectory()
        self.traj_points.clear()
        self.current_segments.clear()

    def _canvas_click(self, x: float, y: float):
        self._log(f"Click at ({x:.1f}, {y:.1f})")
        if self.drawing_mode == "line":
            self.traj_points.append((x, y))
            self.canvas.set_trajectory(self.traj_points)
            if len(self.traj_points) == 2:
                pts = interpolate_line(
                    self.traj_points[0][0], self.traj_points[0][1],
                    self.traj_points[1][0], self.traj_points[1][1],
                    self.step_mm.value()
                )
                self.canvas.temp_points = pts
                self.canvas.update()
                self._log(f"  Line: {len(pts)} interpolated points")
        elif self.drawing_mode == "arc":
            if self.arc_center is None:
                self.arc_center = (x, y)
                self.canvas.temp_points = [(x, y)]
                self.canvas.update()
                self._log(f"  Arc center: ({x:.1f}, {y:.1f}) — click start point")
            elif len(self.traj_points) == 0:
                self.traj_points.append((x, y))
                self.canvas.temp_points = [self.arc_center, (x, y)]
                self.canvas.update()
                self._log(f"  Arc start: ({x:.1f}, {y:.1f}) — click end point")
            else:
                start = self.traj_points[0]
                pts = interpolate_arc(
                    start[0], start[1], x, y,
                    self.arc_center[0], self.arc_center[1],
                    self.ccw_cb.isChecked(),
                    self.step_mm.value()
                )
                if pts:
                    self.traj_points.extend(pts[1:])
                    self.canvas.set_trajectory(self.traj_points)
                    self.canvas.temp_points = pts
                    self.canvas.update()
                    self._log(f"  Arc: {len(pts)} interpolated points")
                self.arc_center = None

    def _clear_traj(self):
        self.traj_points.clear()
        self.arc_center = None
        self.canvas.clear_trajectory()
        self.current_segments.clear()
        self.progress_label.setText("")

    def _preview_traj(self):
        if len(self.traj_points) < 2:
            self._log("Need at least 2 points for trajectory")
            return
        speed = self.speed_slider.value()
        self.current_segments = dda_interpolate_segments(self.traj_points, speed)
        self._log(f"Preview: {len(self.current_segments)} segments, "
                  f"{(len(self.traj_points))} waypoints")

    def _send_trajectory(self):
        if not self.serial.connected:
            QMessageBox.warning(self, "Error", "Not connected to robot")
            return
        if not self.current_segments:
            self._preview_traj()
        segments = self.current_segments
        if not segments:
            self._log("No segments to send")
            return
        self.progress_label.setText(f"Sending 0/{len(segments)}")
        self._log(f"Starting trajectory: {len(segments)} segments")
        for i, seg in enumerate(segments):
            resp = self.serial.move_relative(seg["d1"], seg["d2"], seg["speed"])
            if "OK" not in resp:
                self._log(f"  Segment {i} failed: {resp}")
                self.progress_label.setText(f"Failed at segment {i}")
                return
            self._log(f"  Segment {i}: d1={seg['d1']} d2={seg['d2']}")
            self.progress_label.setText(f"Waiting {i+1}/{len(segments)}")
            QApplication.processEvents()
            if not self.serial.wait_ready(timeout=30.0):
                self._log(f"  Segment {i} timeout")
                self.progress_label.setText("Timeout")
                return
            self.progress_label.setText(f"Sent {i+1}/{len(segments)}")
            QApplication.processEvents()
        self._log("Trajectory complete")
        self.progress_label.setText("Done")

    def _poll_serial(self):
        if not self.serial.connected:
            return
        resp = self.serial.query()
        parsed = self.serial.parse_position(resp)
        if parsed:
            s1, s2, status = parsed
            x, y = steps_to_cartesian(s1, s2)
            from scara_kinematics import steps_to_angle
            t1 = steps_to_angle(s1)
            t2 = steps_to_angle(s2)
            self.canvas.set_angles(t1, t2)
            self.statusBar.showMessage(
                f"Pos: ({x:.1f}, {y:.1f}) mm  "
                f"θ1={t1:.1f}° θ2={t2:.1f}°  Status: {status}"
            )


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = SCARAHost()
    win.show()
    win._refresh_ports()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
