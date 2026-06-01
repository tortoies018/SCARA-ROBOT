import sys
import math
from typing import Optional

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QComboBox, QLabel, QTextEdit, QLineEdit,
    QSplitter, QFormLayout, QDoubleSpinBox, QSpinBox, QSlider,
    QMessageBox, QStatusBar, QGridLayout, QCheckBox, QScrollArea, QFrame
)
from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal, QSize
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QMouseEvent, QPaintEvent,
    QLinearGradient, QRadialGradient, QPalette, QFontDatabase
)

from scara_serial import SCARASerial
from scara_kinematics import (
    forward_kinematics, inverse_kinematics, cartesian_to_steps,
    steps_to_cartesian, interpolate_line, interpolate_arc,
    dda_interpolate_segments, ARM1_LENGTH, ARM2_LENGTH
)

DARK_BG      = "#1e1e2e"
SIDEBAR_BG   = "#181825"
PANEL_BG     = "#252538"
SURFACE_BG   = "#313244"
BORDER_COLOR = "#45475a"
TEXT_PRIMARY = "#cdd6f4"
TEXT_SECOND  = "#a6adc8"
TEXT_DIM     = "#6c7086"
ACCENT       = "#89b4fa"
ACCENT_HOVER = "#b4d0fb"
DANGER       = "#f38ba8"
SUCCESS      = "#a6e3a1"
WARNING      = "#f9e2af"
CANVAS_BG    = "#1e1e2e"

STYLESHEET = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_PRIMARY};
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
    font-size: 13px;
}}
QGroupBox {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_COLOR};
    border-radius: 8px;
    margin-top: 18px;
    padding: 14px 10px 10px 10px;
    font-weight: 600;
    font-size: 12px;
    color: {ACCENT};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 3px 12px;
    margin-left: 10px;
    background-color: {PANEL_BG};
    border-radius: 4px;
    color: {ACCENT};
    font-size: 12px;
}}
QPushButton {{
    background-color: {SURFACE_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER_COLOR};
    border-radius: 6px;
    padding: 6px 14px;
    font-size: 12px;
    font-weight: 500;
    min-height: 22px;
}}
QPushButton:hover {{
    background-color: {BORDER_COLOR};
    border-color: {ACCENT};
    color: {ACCENT_HOVER};
}}
QPushButton:pressed {{
    background-color: {ACCENT};
    color: {DARK_BG};
}}
QPushButton:checked {{
    background-color: {ACCENT};
    color: {DARK_BG};
    border-color: {ACCENT};
}}
QPushButton#stopBtn {{
    background-color: {DANGER};
    color: {DARK_BG};
    border-color: {DANGER};
    font-weight: 700;
}}
QPushButton#stopBtn:hover {{
    background-color: #f5a0b8;
}}
QPushButton#sendBtn {{
    background-color: {SUCCESS};
    color: {DARK_BG};
    border-color: {SUCCESS};
    font-weight: 700;
}}
QPushButton#sendBtn:hover {{
    background-color: #b8edb5;
}}
QComboBox {{
    background-color: {SURFACE_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER_COLOR};
    border-radius: 6px;
    padding: 4px 8px;
    min-height: 22px;
}}
QComboBox:hover {{
    border-color: {ACCENT};
}}
QComboBox::drop-down {{
    border: none;
    width: 24px;
}}
QComboBox QAbstractItemView {{
    background-color: {SURFACE_BG};
    color: {TEXT_PRIMARY};
    selection-background-color: {ACCENT};
    selection-color: {DARK_BG};
    border: 1px solid {BORDER_COLOR};
    border-radius: 4px;
}}
QLabel {{
    color: {TEXT_SECOND};
    font-size: 12px;
}}
QLabel#statusLabel {{
    font-weight: 600;
    font-size: 12px;
}}
QLabel#resultLabel {{
    color: {ACCENT};
    font-weight: 600;
    font-size: 13px;
}}
QLabel#progressLabel {{
    color: {SUCCESS};
    font-weight: 600;
    font-size: 12px;
}}
QLineEdit {{
    background-color: {SURFACE_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER_COLOR};
    border-radius: 6px;
    padding: 6px 10px;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 12px;
}}
QLineEdit:focus {{
    border-color: {ACCENT};
}}
QTextEdit {{
    background-color: {SURFACE_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER_COLOR};
    border-radius: 6px;
    padding: 6px;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 12px;
}}
QDoubleSpinBox, QSpinBox {{
    background-color: {SURFACE_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER_COLOR};
    border-radius: 6px;
    padding: 4px 6px;
    min-height: 22px;
}}
QDoubleSpinBox:focus, QSpinBox:focus {{
    border-color: {ACCENT};
}}
QSlider::groove:horizontal {{
    height: 6px;
    background: {SURFACE_BG};
    border-radius: 3px;
}}
QSlider::handle:horizontal {{
    background: {ACCENT};
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 8px;
}}
QSlider::handle:horizontal:hover {{
    background: {ACCENT_HOVER};
    width: 18px;
    height: 18px;
    margin: -6px 0;
    border-radius: 9px;
}}
QSlider::sub-page:horizontal {{
    background: {ACCENT};
    border-radius: 3px;
}}
QCheckBox {{
    color: {TEXT_SECOND};
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid {BORDER_COLOR};
    border-radius: 3px;
    background-color: {SURFACE_BG};
}}
QCheckBox::indicator:checked {{
    background-color: {ACCENT};
    border-color: {ACCENT};
}}
QScrollBar:vertical {{
    background: {PANEL_BG};
    width: 10px;
    border-radius: 5px;
}}
QScrollBar::handle:vertical {{
    background: {BORDER_COLOR};
    border-radius: 5px;
    min-height: 30px;
}}
QScrollBar::handle:vertical:hover {{
    background: {ACCENT};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QSplitter::handle {{
    background: {BORDER_COLOR};
    width: 2px;
}}
QStatusBar {{
    background-color: {SIDEBAR_BG};
    color: {TEXT_DIM};
    border-top: 1px solid {BORDER_COLOR};
    font-size: 12px;
}}
"""


class SCARAWidget(QWidget):
    mouse_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 400)
        self.setMouseTracking(True)

        self.theta1: float = 0.0
        self.theta2: float = 0.0

        self.traj_points: list[tuple[float, float]] = []
        self.temp_points: list[tuple[float, float]] = []
        self.home_pos: tuple[float, float] = (ARM1_LENGTH + ARM2_LENGTH, 0.0)
        self.origin: QPointF = QPointF()

        self.scale: float = 1.5
        self.mouse_pos: Optional[QPointF] = None
        self._pan_offset: QPointF = QPointF(0, 0)
        self._panning = False
        self._pan_start: Optional[QPointF] = None

    def set_angles(self, t1: float, t2: float):
        self.theta1 = t1
        self.theta2 = t2
        self.update()

    def set_trajectory(self, pts: list[tuple[float, float]]):
        self.traj_points = list(pts)
        self.update()

    def clear_trajectory(self):
        self.traj_points.clear()
        self.temp_points.clear()
        self.update()

    def reset_view(self):
        self._pan_offset = QPointF(0, 0)
        self.scale = 1.5
        self.update()

    def paintEvent(self, event: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0 + self._pan_offset.x(), h / 2.0 + self._pan_offset.y()

        p.fillRect(self.rect(), QColor(CANVAS_BG))

        p.setPen(QPen(QColor("#313244"), 1))
        grid = 30
        for x in range(int(cx % grid), w, grid):
            p.drawLine(x, 0, x, h)
        for y in range(int(cy % grid), h, grid):
            p.drawLine(0, y, w, y)

        p.setPen(QPen(QColor("#45475a"), 1))
        p.drawLine(0, int(cy), w, int(cy))
        p.drawLine(int(cx), 0, int(cx), h)

        font_small = QFont("Segoe UI", 9)
        p.setFont(font_small)
        p.setPen(QColor(TEXT_DIM))
        for x in range(int(cx % 100) if cx % 100 < 50 else int(cx % 100) - 100, w, 100):
            if abs(x - int(cx)) > 5:
                p.drawText(x - 12, int(cy) + 16, f"{x - int(cx)}")
        for y in range(int(cy % 100) if cy % 100 < 50 else int(cy % 100) - 100, h, 100):
            if abs(y - int(cy)) > 5:
                p.drawText(int(cx) + 6, y + 4, f"{int(cy) - y}")

        p.save()
        p.translate(cx, cy)
        p.scale(1, -1)
        p.scale(self.scale, self.scale)
        ss = self.scale

        if self.temp_points:
            pen = QPen(QColor(ACCENT), 1.8 / ss, Qt.PenStyle.DashLine)
            pen.setDashPattern([4 / ss, 3 / ss])
            p.setPen(pen)
            for i in range(len(self.temp_points) - 1):
                x1, y1 = self.temp_points[i]
                x2, y2 = self.temp_points[i + 1]
                p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
            for pt in self.temp_points:
                p.setBrush(QBrush(QColor(ACCENT)))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(QPointF(pt[0], pt[1]), 3.0 / ss, 3.0 / ss)

        if self.traj_points:
            p.setPen(QPen(QColor(ACCENT), 2.0 / ss))
            p.setBrush(Qt.BrushStyle.NoBrush)
            for i in range(len(self.traj_points) - 1):
                x1, y1 = self.traj_points[i]
                x2, y2 = self.traj_points[i + 1]
                p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
            for pt in self.traj_points:
                p.setBrush(QBrush(QColor(SUCCESS)))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(QPointF(pt[0], pt[1]), 3.5 / ss, 3.5 / ss)

        home = QPointF(self.home_pos[0], self.home_pos[1])
        p.setBrush(QBrush(QColor(WARNING)))
        p.setPen(QPen(QColor(DARK_BG), 1.5 / ss))
        p.drawEllipse(home, 4.0 / ss, 4.0 / ss)
        p.setPen(QColor(TEXT_DIM))
        p.drawText(home + QPointF(-12, -12) / ss, "HOME")

        r1 = math.radians(self.theta1)
        r2 = math.radians(self.theta2)
        j1_x = ARM1_LENGTH * math.cos(r1)
        j1_y = ARM1_LENGTH * math.sin(r1)
        ee_x = ARM1_LENGTH * math.cos(r1) + ARM2_LENGTH * math.cos(r1 + r2)
        ee_y = ARM1_LENGTH * math.sin(r1) + ARM2_LENGTH * math.sin(r1 + r2)

        p.setPen(QPen(QColor("#f38ba8"), 5.0 / ss, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(0, 0), QPointF(j1_x, j1_y))
        p.setPen(QPen(QColor("#89b4fa"), 5.0 / ss, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(j1_x, j1_y), QPointF(ee_x, ee_y))

        p.setBrush(QBrush(QColor("#f38ba8")))
        p.setPen(QPen(QColor(DARK_BG), 1.5 / ss))
        p.drawEllipse(QPointF(0, 0), 5.0 / ss, 5.0 / ss)

        p.setBrush(QBrush(QColor("#a6e3a1")))
        p.drawEllipse(QPointF(j1_x, j1_y), 4.0 / ss, 4.0 / ss)

        p.setBrush(QBrush(QColor("#89b4fa")))
        p.drawEllipse(QPointF(ee_x, ee_y), 4.5 / ss, 4.5 / ss)

        font_label = QFont("Segoe UI", 11)
        font_label.setPixelSize(int(12 / ss * self.scale))
        p.setFont(font_label)
        p.setPen(QColor(TEXT_PRIMARY))
        p.drawText(QPointF(-18, 0) / ss, "●")
        p.drawText(QPointF(j1_x, j1_y) / ss + QPointF(0, -16) / ss, "J1")
        p.drawText(QPointF(ee_x, ee_y) / ss + QPointF(0, 16) / ss,
                   f"({ee_x:.1f}, {ee_y:.1f})")

        if self.mouse_pos:
            mp = self.mouse_pos
            p.setPen(QPen(QColor(ACCENT), 0.8 / ss, Qt.PenStyle.DashLine))
            p.drawLine(QPointF(mp.x(), -500), QPointF(mp.x(), 500))
            p.drawLine(QPointF(-500, mp.y()), QPointF(500, mp.y()))
            p.setBrush(QBrush(QColor(ACCENT)))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(mp.x(), mp.y()), 2.5 / ss, 2.5 / ss)

        p.restore()

        p.setPen(QColor(TEXT_DIM))
        font_info = QFont("Segoe UI", 11)
        p.setFont(font_info)
        p.drawText(14, 24,
                   f"θ₁={self.theta1:.1f}°  θ₂={self.theta2:.1f}°  "
                   f"EE=({ee_x:.1f}, {ee_y:.1f})  [{self.scale:.1f}x]")

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            if event.modifiers() == Qt.KeyboardModifier.ControlModifier:
                self._panning = True
                self._pan_start = event.position()
            else:
                wpos = self._widget_to_scene(event.position())
                self.mouse_clicked.emit(wpos.x(), wpos.y())
        elif event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_start = event.position()

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._panning and self._pan_start:
            delta = event.position() - self._pan_start
            self._pan_offset += delta
            self._pan_start = event.position()
            self.update()
            return
        wpos = self._widget_to_scene(event.position())
        self.mouse_pos = QPointF(wpos.x(), wpos.y())
        self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        self._panning = False
        self._pan_start = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        factor = 1.08 if delta > 0 else 1 / 1.08
        self.scale = max(0.2, min(20.0, self.scale * factor))
        self.update()

    def _widget_to_scene(self, pos: QPointF) -> QPointF:
        w, h = self.width(), self.height()
        cx = w / 2.0 + self._pan_offset.x()
        cy = h / 2.0 + self._pan_offset.y()
        return QPointF((pos.x() - cx) / self.scale, (cy - pos.y()) / self.scale)

    def resizeEvent(self, event):
        self.update()
        super().resizeEvent(event)


class SidebarSection(QGroupBox):
    def __init__(self, title: str, parent=None):
        super().__init__(title, parent)
        self.content_layout = QVBoxLayout(self)
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout.setSpacing(6)


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

        self.setWindowTitle("SCARA 机器人控制器")
        self.resize(1400, 860)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        sidebar = QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(320)
        sidebar.setStyleSheet(f"QWidget#sidebar {{ background-color: {SIDEBAR_BG}; "
                               f"border-right: 1px solid {BORDER_COLOR}; }}")
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_layout.setSpacing(0)

        scroll = _ScrollableContent()
        sidebar_layout.addWidget(scroll, 1)

        self._build_connection(scroll)
        self._build_controls(scroll)
        self._build_forward_kinematics(scroll)
        self._build_inverse_kinematics(scroll)
        self._build_trajectory(scroll)
        scroll.addStretch()

        content_area = QWidget()
        content_area.setStyleSheet(f"background-color: {DARK_BG};")
        content_layout = QVBoxLayout(content_area)
        content_layout.setContentsMargins(8, 8, 8, 8)
        content_layout.setSpacing(8)

        self.canvas = SCARAWidget()
        self.canvas.mouse_clicked.connect(self._canvas_click)
        content_layout.addWidget(self.canvas, 1)

        console_frame = QFrame()
        console_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {SURFACE_BG};
                border: 1px solid {BORDER_COLOR};
                border-radius: 8px;
            }}
        """)
        console_layout = QVBoxLayout(console_frame)
        console_layout.setContentsMargins(8, 8, 8, 8)
        console_layout.setSpacing(4)
        console_header = QLabel("控制台")
        console_header.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px; "
                                      "font-weight: 600; letter-spacing: 0.3px;")
        console_layout.addWidget(console_header)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(160)
        console_layout.addWidget(self.console)
        cmd_row = QHBoxLayout()
        cmd_row.setSpacing(6)
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("raw command...")
        self.cmd_input.returnPressed.connect(self._send_raw_cmd)
        cmd_row.addWidget(self.cmd_input, 1)
        self.cmd_send_btn = QPushButton("发送")
        self.cmd_send_btn.clicked.connect(self._send_raw_cmd)
        cmd_row.addWidget(self.cmd_send_btn)
        console_layout.addLayout(cmd_row)
        content_layout.addWidget(console_frame)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(0)
        splitter.addWidget(sidebar)
        splitter.addWidget(content_area)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        main_layout.addWidget(splitter)

        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("就绪")

    def _build_connection(self, scroll):
        sec = SidebarSection("连接")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(160)
        sec.content_layout.addWidget(self.port_combo)

        row = QHBoxLayout()
        row.setSpacing(6)
        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setToolTip("刷新端口")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        row.addWidget(self.refresh_btn)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baud_combo.setCurrentText("115200")
        row.addWidget(self.baud_combo, 1)
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self._toggle_connect)
        row.addWidget(self.connect_btn)
        sec.content_layout.addLayout(row)

        self.conn_status = QLabel("● 未连接")
        self.conn_status.setObjectName("statusLabel")
        self.conn_status.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")
        sec.content_layout.addWidget(self.conn_status)
        scroll.add_section(sec)

    def _build_controls(self, scroll):
        sec = SidebarSection("机器人控制")
        grid = QGridLayout()
        grid.setSpacing(6)
        self.home_btn = QPushButton("🏠 回零")
        self.home_btn.clicked.connect(self._cmd_home)
        grid.addWidget(self.home_btn, 0, 0)
        self.pen_up_btn = QPushButton("✏ 抬笔")
        self.pen_up_btn.clicked.connect(lambda: self._send_cmd("P0\r\n"))
        grid.addWidget(self.pen_up_btn, 0, 1)
        self.pen_down_btn = QPushButton("✏ 下笔")
        self.pen_down_btn.clicked.connect(lambda: self._send_cmd("P1\r\n"))
        grid.addWidget(self.pen_down_btn, 0, 2)
        self.stop_btn = QPushButton("⏹ 停止")
        self.stop_btn.setObjectName("stopBtn")
        self.stop_btn.clicked.connect(self._cmd_stop)
        grid.addWidget(self.stop_btn, 1, 0, 1, 3)
        self.query_btn = QPushButton("⟳ 查询")
        self.query_btn.clicked.connect(self._cmd_query)
        grid.addWidget(self.query_btn, 2, 0, 1, 3)
        sec.content_layout.addLayout(grid)
        scroll.add_section(sec)

    def _build_forward_kinematics(self, scroll):
        sec = SidebarSection("正运动学")
        form = QFormLayout()
        form.setSpacing(6)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self.fk_t1 = QDoubleSpinBox()
        self.fk_t1.setRange(-180, 180)
        self.fk_t1.setSuffix("°")
        self.fk_t1.setDecimals(1)
        form.addRow("θ₁:", self.fk_t1)
        self.fk_t2 = QDoubleSpinBox()
        self.fk_t2.setRange(-180, 180)
        self.fk_t2.setSuffix("°")
        self.fk_t2.setDecimals(1)
        form.addRow("θ₂:", self.fk_t2)
        self.fk_btn = QPushButton("计算正解")
        self.fk_btn.clicked.connect(self._compute_fk)
        form.addRow("", self.fk_btn)
        self.fk_result = QLabel("x: —  y: —")
        self.fk_result.setObjectName("resultLabel")
        form.addRow("", self.fk_result)
        self.fk_spd = QSpinBox()
        self.fk_spd.setRange(10, 720)
        self.fk_spd.setValue(90)
        self.fk_spd.setSingleStep(10)
        self.fk_spd.setSuffix(" °/s")
        form.addRow("速度:", self.fk_spd)
        self.fk_send_btn = QPushButton("运动 →")
        self.fk_send_btn.clicked.connect(self._fk_send)
        form.addRow("", self.fk_send_btn)
        sec.content_layout.addLayout(form)
        scroll.add_section(sec)

    def _build_inverse_kinematics(self, scroll):
        sec = SidebarSection("逆运动学")
        form = QFormLayout()
        form.setSpacing(6)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self.ik_x = QDoubleSpinBox()
        self.ik_x.setRange(-300, 300)
        self.ik_x.setDecimals(1)
        self.ik_x.setSuffix(" mm")
        form.addRow("X:", self.ik_x)
        self.ik_y = QDoubleSpinBox()
        self.ik_y.setRange(-300, 300)
        self.ik_y.setDecimals(1)
        self.ik_y.setSuffix(" mm")
        form.addRow("Y:", self.ik_y)
        self.ik_btn = QPushButton("计算逆解")
        self.ik_btn.clicked.connect(self._compute_ik)
        form.addRow("", self.ik_btn)
        self.ik_result = QLabel("θ₁: —  θ₂: —")
        self.ik_result.setObjectName("resultLabel")
        form.addRow("", self.ik_result)
        self.ik_spd = QSpinBox()
        self.ik_spd.setRange(10, 720)
        self.ik_spd.setValue(90)
        self.ik_spd.setSingleStep(10)
        self.ik_spd.setSuffix(" °/s")
        form.addRow("速度:", self.ik_spd)
        self.ik_send_btn = QPushButton("运动 →")
        self.ik_send_btn.clicked.connect(self._ik_send)
        form.addRow("", self.ik_send_btn)
        sec.content_layout.addLayout(form)
        scroll.add_section(sec)

    def _build_trajectory(self, scroll):
        sec = SidebarSection("轨迹规划")
        mode_row = QHBoxLayout()
        mode_row.setSpacing(6)
        self.line_btn = QPushButton("直线")
        self.line_btn.setCheckable(True)
        self.line_btn.setChecked(True)
        self.line_btn.clicked.connect(lambda: self._set_draw_mode("line"))
        self.arc_btn = QPushButton("圆弧")
        self.arc_btn.setCheckable(True)
        self.arc_btn.clicked.connect(lambda: self._set_draw_mode("arc"))
        mode_row.addWidget(self.line_btn)
        mode_row.addWidget(self.arc_btn)
        self.ccw_cb = QCheckBox("逆时针")
        self.ccw_cb.setChecked(True)
        self.ccw_cb.setEnabled(False)
        mode_row.addWidget(self.ccw_cb)
        sec.content_layout.addLayout(mode_row)

        def set_mode(mode):
            self._set_draw_mode(mode)
            self.arc_btn.setChecked(mode == "arc")
            self.line_btn.setChecked(mode == "line")
            self.ccw_cb.setEnabled(mode == "arc")

        self.line_btn.clicked.disconnect()
        self.line_btn.clicked.connect(lambda: set_mode("line"))
        self.arc_btn.clicked.disconnect()
        self.arc_btn.clicked.connect(lambda: set_mode("arc"))

        spd_row = QHBoxLayout()
        spd_row.setSpacing(6)
        spd_row.addWidget(QLabel("速度:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(10, 720)
        self.speed_slider.setValue(90)
        spd_row.addWidget(self.speed_slider, 1)
        self.speed_label = QLabel("90")
        self.speed_label.setStyleSheet(f"color: {ACCENT}; font-weight: 600; "
                                        "font-family: 'Consolas', monospace;")
        spd_row.addWidget(self.speed_label)
        self.speed_label_suffix = QLabel("°/s")
        self.speed_label_suffix.setStyleSheet(f"color: {TEXT_DIM};")
        spd_row.addWidget(self.speed_label_suffix)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(str(v)))
        self.speed_slider.sliderReleased.connect(self._on_speed_changed)
        self._speed_debounce = QTimer()
        self._speed_debounce.setSingleShot(True)
        self._speed_debounce.timeout.connect(self._send_speed_to_firmware)
        sec.content_layout.addLayout(spd_row)

        stp_row = QHBoxLayout()
        stp_row.setSpacing(6)
        stp_row.addWidget(QLabel("步长:"))
        self.step_mm = QDoubleSpinBox()
        self.step_mm.setRange(0.1, 10.0)
        self.step_mm.setValue(1.0)
        self.step_mm.setSingleStep(0.1)
        self.step_mm.setSuffix(" mm")
        stp_row.addWidget(self.step_mm, 1)
        sec.content_layout.addLayout(stp_row)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        self.clear_btn = QPushButton("清空")
        self.clear_btn.clicked.connect(self._clear_traj)
        btn_row.addWidget(self.clear_btn)
        self.reset_view_btn = QPushButton("⊡ 复位")
        self.reset_view_btn.clicked.connect(lambda: self.canvas.reset_view())
        btn_row.addWidget(self.reset_view_btn)
        self.preview_btn = QPushButton("预览")
        self.preview_btn.clicked.connect(self._preview_traj)
        btn_row.addWidget(self.preview_btn)
        sec.content_layout.addLayout(btn_row)

        self.send_traj_btn = QPushButton("▶ 发送轨迹")
        self.send_traj_btn.setObjectName("sendBtn")
        self.send_traj_btn.clicked.connect(self._send_trajectory)
        sec.content_layout.addWidget(self.send_traj_btn)

        self.progress_label = QLabel("")
        self.progress_label.setObjectName("progressLabel")
        sec.content_layout.addWidget(self.progress_label)

        scroll.add_section(sec)

    def _setup_timers(self):
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self._poll_serial)
        self._poll_timer.start(200)

    def _on_speed_changed(self):
        self._speed_debounce.start(150)

    def _send_speed_to_firmware(self):
        if self.serial.connected:
            from scara_kinematics import deg_per_sec_to_hz
            speed_hz = deg_per_sec_to_hz(self.speed_slider.value())
            self.serial.set_speed(speed_hz)

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in self.serial.list_ports():
            self.port_combo.addItem(p)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found")

    def _toggle_connect(self):
        if self.serial.connected:
            self.serial.disconnect()
            self.connect_btn.setText("连接")
            self.conn_status.setText("● 未连接")
            self.conn_status.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")
        else:
            port = self.port_combo.currentText()
            if not port or port == "No ports found":
                return
            baud = int(self.baud_combo.currentText())
            if self.serial.connect(port, baud):
                self.connect_btn.setText("断开")
                self.conn_status.setText("● 已连接")
                self.conn_status.setStyleSheet(f"color: {SUCCESS}; font-size: 12px; font-weight: 600;")
                self._log("已连接 " + port)
                self._send_speed_to_firmware()
            else:
                QMessageBox.warning(self, "错误", f"连接失败: {port}")

    def _log(self, msg: str):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.console.append(
            f'<span style="color:{TEXT_DIM}">[{ts}]</span> '
            f'<span style="color:{TEXT_PRIMARY}">{msg}</span>'
        )

    def _send_raw_cmd(self):
        txt = self.cmd_input.text().strip()
        if not txt:
            return
        self.cmd_input.clear()
        self._log(f'<span style="color:{ACCENT}">→ {txt}</span>')
        resp = self.serial.send(txt + "\r\n")
        self._log(f'<span style="color:{WARNING}">← {resp}</span>')

    def _send_cmd(self, cmd: str):
        self._log(f'<span style="color:{ACCENT}">→ {cmd.strip()}</span>')
        resp = self.serial.send(cmd)
        self._log(f'<span style="color:{WARNING}">← {resp}</span>')

    def _cmd_home(self):
        self._send_cmd("H\r\n")

    def _cmd_stop(self):
        self._send_cmd("!\r\n")

    def _cmd_query(self):
        resp = self.serial.query()
        self._log(f'<span style="color:{ACCENT}">→ Q</span>')
        self._log(f'<span style="color:{WARNING}">← {resp}</span>')
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
        self._log(f"FK: θ₁={t1:.1f}° θ₂={t2:.1f}° → ({x:.2f}, {y:.2f})")

    def _fk_send(self):
        t1 = self.fk_t1.value()
        t2 = self.fk_t2.value()
        from scara_kinematics import angle_to_steps, deg_per_sec_to_hz
        s1 = angle_to_steps(t1)
        s2 = angle_to_steps(t2)
        speed_hz = deg_per_sec_to_hz(self.fk_spd.value())
        resp = self.serial.move_absolute(s1, s2, speed_hz)
        self._log(f"→ A {s1} {s2} {speed_hz}Hz ({self.fk_spd.value()}°/s)")
        self._log(f"← {resp}")

    def _compute_ik(self):
        x = self.ik_x.value()
        y = self.ik_y.value()
        ik = inverse_kinematics(x, y)
        if ik is None:
            self.ik_result.setText("不可达")
            self._log("IK: 位置不可达")
            return
        t1, t2 = ik
        self.ik_result.setText(f"θ₁: {t1:.2f}°  θ₂: {t2:.2f}°")
        self.canvas.set_angles(t1, t2)
        self._log(f"IK: ({x:.2f}, {y:.2f}) → θ₁={t1:.2f}° θ₂={t2:.2f}°")

    def _ik_send(self):
        x = self.ik_x.value()
        y = self.ik_y.value()
        ik = inverse_kinematics(x, y)
        if ik is None:
            self._log("IK: 位置不可达")
            return
        t1, t2 = ik
        from scara_kinematics import angle_to_steps, deg_per_sec_to_hz
        s1 = angle_to_steps(t1)
        s2 = angle_to_steps(t2)
        speed_hz = deg_per_sec_to_hz(self.ik_spd.value())
        resp = self.serial.move_absolute(s1, s2, speed_hz)
        self._log(f"→ A {s1} {s2} {speed_hz}Hz ({self.ik_spd.value()}°/s)")
        self._log(f"← {resp}")

    def _set_draw_mode(self, mode: str):
        self.drawing_mode = mode
        self.line_btn.setChecked(mode == "line")
        self.arc_btn.setChecked(mode == "arc")
        self.ccw_cb.setEnabled(mode == "arc")
        self.arc_center = None
        self.canvas.clear_trajectory()
        self.traj_points.clear()
        self.current_segments.clear()

    def _canvas_click(self, x: float, y: float):
        self._log(f"点击 ({x:.1f}, {y:.1f})")
        if self.drawing_mode == "line":
            self.traj_points.append((x, y))
            self.canvas.set_trajectory(self.traj_points)
            if len(self.traj_points) >= 2:
                pts = interpolate_line(
                    self.traj_points[-2][0], self.traj_points[-2][1],
                    self.traj_points[-1][0], self.traj_points[-1][1],
                    self.step_mm.value()
                )
                self.canvas.temp_points = pts
                self.canvas.update()
                self._log(f"  → {len(pts)} 点")
        elif self.drawing_mode == "arc":
            if self.arc_center is None:
                self.arc_center = (x, y)
                self.canvas.temp_points = [(x, y)]
                self.canvas.update()
                self._log("  圆心已设 — 点开始")
            elif len(self.traj_points) == 0:
                self.traj_points.append((x, y))
                self.canvas.temp_points = [self.arc_center, (x, y)]
                self.canvas.update()
                self._log("  起点已设 — 点终点")
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
                    self._log(f"  圆弧: {len(pts)} 点")
                self.arc_center = None

    def _clear_traj(self):
        self.traj_points.clear()
        self.arc_center = None
        self.canvas.clear_trajectory()
        self.current_segments.clear()
        self.progress_label.setText("")

    def _preview_traj(self):
        if len(self.traj_points) < 2:
            self._log("需要 ≥ 2 点")
            return
        speed_deg_s = self.speed_slider.value()
        self.current_segments = dda_interpolate_segments(self.traj_points, speed_deg_s)
        self._log(f"预览: {len(self.current_segments)} 段 (速度 {speed_deg_s}°/s)")

    def _send_trajectory(self):
        if not self.serial.connected:
            QMessageBox.warning(self, "错误", "未连接机器人")
            return
        if not self.current_segments:
            self._preview_traj()
        segments = self.current_segments
        if not segments:
            self._log("无轨迹段")
            return
        self.progress_label.setText(f"⏳ 0/{len(segments)}")
        self._log(f"轨迹: {len(segments)} 段")
        for i, seg in enumerate(segments):
            resp = self.serial.move_relative(seg["d1"], seg["d2"], seg["speed"])
            if "OK" not in resp:
                self._log(f"  ✗ 段 {i}: {resp}")
                self.progress_label.setText(f"✗ 失败")
                return
            self.progress_label.setText(f"⏳ {i+1}/{len(segments)}")
            QApplication.processEvents()
            if not self.serial.wait_ready(timeout=30.0):
                self._log(f"  ✗ 段 {i} 超时")
                self.progress_label.setText("✗ 超时")
                return
            self.progress_label.setText(f"✓ {i+1}/{len(segments)}")
            QApplication.processEvents()
        self._log("✓ 轨迹完成")
        self.progress_label.setText("✓ 完成")

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
            emoji = "●" if "BSY" in status else "○"
            clr = WARNING if "BSY" in status else SUCCESS
            self.statusBar.showMessage(
                f'<span style="color:{clr}">{emoji}</span> '
                f'Pos ({x:.1f}, {y:.1f}) mm  '
                f'θ₁={t1:.1f}°  θ₂={t2:.1f}°  '
                f'Status: {status}'
            )


class _ScrollableContent(QScrollArea):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setStyleSheet(f"QScrollArea {{ border: none; background: {SIDEBAR_BG}; }}")
        self._content = QWidget()
        self._content.setStyleSheet(f"background: {SIDEBAR_BG};")
        self._layout = QVBoxLayout(self._content)
        self._layout.setContentsMargins(8, 4, 8, 8)
        self._layout.setSpacing(4)
        self.setWidget(self._content)

    def add_section(self, section: QWidget):
        self._layout.addWidget(section)

    def addStretch(self):
        self._layout.addStretch()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLESHEET)
    font = QFont("Segoe UI", 10)
    font.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
    app.setFont(font)
    win = SCARAHost()
    win.show()
    win._refresh_ports()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
