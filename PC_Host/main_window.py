"""
SCARA 并联机器人控制主窗口
双摇杆五连杆结构: 主动臂110mm, 从动臂220mm, 电机间距160mm
"""
import math
from PyQt6.QtCore import QTimer, Qt, QPointF
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QComboBox, QLabel, QTextEdit, QLineEdit,
    QFormLayout, QDoubleSpinBox, QStatusBar, QFrame, QGridLayout
)
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QMouseEvent
from serial_worker import SerialWorker, list_ports

# ======================== 机械参数 ========================
L1 = 110.0      # 主动臂长 mm
L2 = 220.0      # 从动臂长 mm
D  = 160.0      # 电机间距 mm
HALF_D = D / 2  # 半间距

def forward_kinematics(deg1: float, deg2: float) -> tuple[float, float] | None:
    """正运动学: 两主动臂角度 → 末端坐标 (两圆交点)"""
    r1, r2 = math.radians(deg1), math.radians(deg2)
    # 主动臂末端 (两个圆的圆心)
    p1x = -HALF_D + L1 * math.cos(r1)
    p1y =          L1 * math.sin(r1)
    p2x =  HALF_D + L1 * math.cos(r2)
    p2y =          L1 * math.sin(r2)
    # 两圆相交求末端
    dx = p2x - p1x
    dy = p2y - p1y
    d2 = dx * dx + dy * dy
    if d2 > (2 * L2) ** 2 or d2 < 0.001:
        return None  # 无解或重合
    d = math.sqrt(d2)
    a = d / 2
    h2 = L2 * L2 - a * a
    if h2 < 0:
        return None
    h = math.sqrt(h2)
    mx = (p1x + p2x) / 2
    my = (p1y + p2y) / 2
    # 取 y 较小的交点 (下方)
    ex = mx - h * dy / d
    ey = my + h * dx / d
    return ex, ey

def inverse_kinematics(x: float, y: float) -> tuple[float, float] | None:
    """逆运动学: 末端坐标 → 两主动臂角度 (elbow-down)"""
    # 电机1 (-HALF_D, 0)
    r1 = math.hypot(x + HALF_D, y)
    if r1 > L1 + L2 or r1 < abs(L2 - L1):
        return None
    cos_phi1 = (L1 * L1 + r1 * r1 - L2 * L2) / (2 * L1 * r1)
    phi1 = math.acos(max(-1, min(1, cos_phi1)))
    t1 = math.atan2(y, x + HALF_D) - phi1  # elbow-down

    # 电机2 (HALF_D, 0)
    r2 = math.hypot(x - HALF_D, y)
    if r2 > L1 + L2 or r2 < abs(L2 - L1):
        return None
    cos_phi2 = (L1 * L1 + r2 * r2 - L2 * L2) / (2 * L1 * r2)
    phi2 = math.acos(max(-1, min(1, cos_phi2)))
    t2 = math.pi - (math.atan2(y, x - HALF_D) + phi2)  # elbow-down

    return math.degrees(t1), math.degrees(t2)

# ======================== Canvas ========================
class Canvas(QWidget):
    """并联机械臂 2D 画布 (可拖动/缩放)"""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(500, 400)
        self.setMouseTracking(True)
        self.t1 = -60.0     # 主动臂1角度
        self.t2 = -120.0    # 主动臂2角度
        self.scale = 1.8
        self.pan = QPointF(0, 0)     # 平移偏移
        self._mouse = QPointF(0, 0)  # 鼠标位置
        self._drag = False
        self._drag_start = QPointF(0, 0)

    def set_angles(self, t1: float, t2: float):
        self.t1 = t1
        self.t2 = t2
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2 + self.pan.x(), h / 2 + self.pan.y()

        # 背景
        p.fillRect(self.rect(), QColor("#1e1e2e"))

        # 网格 (30px)
        p.setPen(QPen(QColor("#2a2a3c"), 1))
        for x in range(int(cx % 30), w, 30): p.drawLine(x, 0, x, h)
        for y in range(int(cy % 30), h, 30): p.drawLine(0, y, w, y)

        # 坐标轴
        p.setPen(QPen(QColor("#3a3a50"), 1))
        p.drawLine(0, int(cy), w, int(cy))
        p.drawLine(int(cx), 0, int(cx), h)

        # 刻度标签
        f = QFont("Segoe UI", 9); p.setFont(f); p.setPen(QColor("#585b70"))
        for x in range(int(cx % 100), w, 100):
            if abs(x - int(cx)) > 15: p.drawText(x - 10, int(cy) + 14, f"{x - int(cx)}")
        for y in range(int(cy % 100), h, 100):
            if abs(y - int(cy)) > 15: p.drawText(int(cx) + 6, y + 4, f"{int(cy) - y}")

        # 原点标签
        p.setPen(QColor("#585b70"))
        p.drawText(int(cx) + 6, int(cy) + 14, "O")

        # ====== 坐标变换 ======
        p.save()
        p.translate(cx, cy)
        p.scale(1, -1)
        p.scale(self.scale, self.scale)
        s = self.scale

        # 计算机构位置
        r1 = math.radians(self.t1)
        r2 = math.radians(self.t2)
        # 主动臂末端
        a1x = -HALF_D + L1 * math.cos(r1);  a1y = L1 * math.sin(r1)
        a2x =  HALF_D + L1 * math.cos(r2);  a2y = L1 * math.sin(r2)
        # 末端 (两圆交点)
        ee = forward_kinematics(self.t1, self.t2)
        if ee:
            ex, ey = ee
        else:
            ex, ey = 0, -L2

        # ----- 网格 (mm) -----
        p.setPen(QPen(QColor("#2a2a3c"), 1 / s))
        for g in range(-500, 501, 50):
            p.drawLine(QPointF(-500 / s, g / s), QPointF(500 / s, g / s))
            p.drawLine(QPointF(g / s, -500 / s), QPointF(g / s, 500 / s))

        # ----- 从动臂 (虚线) -----
        p.setPen(QPen(QColor("#6c7086"), 2 / s, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(a1x, a1y), QPointF(ex, ey))
        p.drawLine(QPointF(a2x, a2y), QPointF(ex, ey))

        # ----- 主动臂 -----
        # 臂1 (左)
        p.setPen(QPen(QColor("#f38ba8"), 5 / s, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(-HALF_D, 0), QPointF(a1x, a1y))
        # 臂2 (右)
        p.setPen(QPen(QColor("#89b4fa"), 5 / s, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(HALF_D, 0), QPointF(a2x, a2y))

        # ----- 关节圆点 -----
        p.setBrush(QBrush(QColor("#45475a"))); p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(-HALF_D, 0), 6 / s, 6 / s)   # 电机1底座
        p.drawEllipse(QPointF(HALF_D, 0), 6 / s, 6 / s)    # 电机2底座

        p.setBrush(QBrush(QColor("#f38ba8")))
        p.drawEllipse(QPointF(a1x, a1y), 4.5 / s, 4.5 / s)  # 主动臂1末端

        p.setBrush(QBrush(QColor("#89b4fa")))
        p.drawEllipse(QPointF(a2x, a2y), 4.5 / s, 4.5 / s)  # 主动臂2末端

        p.setBrush(QBrush(QColor("#a6e3a1")))
        p.drawEllipse(QPointF(ex, ey), 5 / s, 5 / s)        # 末端

        # ----- 电机间距虚线 -----
        p.setPen(QPen(QColor("#fab387"), 1.5 / s, Qt.PenStyle.DotLine))
        p.drawLine(QPointF(-HALF_D, 0), QPointF(HALF_D, 0))

        p.restore()

        # ====== 文字标签 (在 restore 之后绘制, 避免翻转) ======
        def sx(x): return cx + x * self.scale
        def sy(y): return cy - y * self.scale

        f2 = QFont("Segoe UI", 10); p.setFont(f2)

        p.setPen(QColor("#f38ba8"))
        p.drawText(int(sx(-HALF_D)) - 14, int(sy(0)) + 4, "M₁")

        p.setPen(QColor("#89b4fa"))
        p.drawText(int(sx(HALF_D)) + 6, int(sy(0)) + 4, "M₂")

        p.setPen(QColor("#a6e3a1"))
        p.drawText(int(sx(ex)) + 8, int(sy(ey)) + 4, f"({ex:.0f}, {ey:.0f})")

        p.setPen(QColor("#fab387"))
        p.drawText(int(sx(0)) - 14, int(sy(0)) + 20, f"{D}mm")

        # ====== HUD ======
        p.setPen(QColor("#a6adc8")); p.setFont(QFont("Segoe UI", 11))
        p.drawText(14, 24,
            f"θ₁={self.t1:.1f}°  θ₂={self.t2:.1f}°  "
            f"EE=({(ee[0] if ee else 0):.1f}, {(ee[1] if ee else 0):.1f})  "
            f"[{self.scale:.1f}x]")

    # ====== 交互 ======
    def mousePressEvent(self, e: QMouseEvent):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drag = True
            self._drag_start = e.position()
        elif e.button() == Qt.MouseButton.MiddleButton:
            self._drag = True
            self._drag_start = e.position()

    def mouseMoveEvent(self, e: QMouseEvent):
        if self._drag:
            delta = e.position() - self._drag_start
            self.pan += delta
            self._drag_start = e.position()
            self.update()
        else:
            w, h = self.width(), self.height()
            cx, cy = w / 2 + self.pan.x(), h / 2 + self.pan.y()
            self._mouse = QPointF(
                (e.position().x() - cx) / self.scale,
                (cy - e.position().y()) / self.scale
            )
            self.update()

    def mouseReleaseEvent(self, e: QMouseEvent):
        self._drag = False

    def wheelEvent(self, event):
        f = 1.08 if event.angleDelta().y() > 0 else 1 / 1.08
        self.scale = max(0.3, min(15, self.scale * f))
        self.update()

    def resizeEvent(self, event):
        self.update()
        super().resizeEvent(event)


# ======================== 主窗口 ========================
class MainW(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial = SerialWorker()
        self._rx = ""
        self._build_ui()
        self.serial.signals.data_received.connect(self._on_data)
        self.serial.signals.connection_status.connect(self._on_status)
        self.setWindowTitle("SCARA 并联机器人控制")
        self.resize(960, 680)
        self.canvas.set_angles(90, 90)

    def _build_ui(self):
        cw = QWidget(); self.setCentralWidget(cw)
        ml = QHBoxLayout(cw); ml.setContentsMargins(8, 8, 8, 8)

        # ---- 侧栏 ----
        left = QWidget(); ll = QVBoxLayout(left); ll.setSpacing(6)

        sg = QGroupBox("串口")
        sl = QGridLayout(sg)
        sl.addWidget(QLabel("端口:"), 0, 0)
        self.port = QComboBox(); sl.addWidget(self.port, 0, 1)
        self.rf = QPushButton("刷新"); self.rf.clicked.connect(self._ref); sl.addWidget(self.rf, 0, 2)
        sl.addWidget(QLabel("波特率:"), 1, 0)
        self.baud = QComboBox(); self.baud.addItems(["9600","19200","38400","57600","115200","230400"])
        self.baud.setCurrentText("115200"); sl.addWidget(self.baud, 1, 1)
        self.cb = QPushButton("连接"); self.cb.clicked.connect(self._tog); sl.addWidget(self.cb, 1, 2)
        self.st = QLabel("● 未连接"); sl.addWidget(self.st, 2, 0, 1, 3)
        ll.addWidget(sg)

        # 电机使能
        eg = QGroupBox("电机")
        el = QHBoxLayout(eg)
        self.en_btn = QPushButton("使能"); self.en_btn.clicked.connect(lambda: self._cmd(f"E 1"))
        self.dis_btn = QPushButton("禁能"); self.dis_btn.clicked.connect(lambda: self._cmd(f"E 0"))
        el.addWidget(self.en_btn); el.addWidget(self.dis_btn)
        ll.addWidget(eg)

        # 正运动学
        fg = QGroupBox("正运动学")
        fl = QFormLayout(fg); fl.setSpacing(6)
        self.fk1 = QDoubleSpinBox(); self.fk1.setRange(-180, 180); self.fk1.setSuffix("°"); self.fk1.setDecimals(1)
        self.fk1.setValue(90)
        fl.addRow("θ₁ (左):", self.fk1)
        self.fk2 = QDoubleSpinBox(); self.fk2.setRange(-180, 180); self.fk2.setSuffix("°"); self.fk2.setDecimals(1)
        self.fk2.setValue(90)
        fl.addRow("θ₂ (右):", self.fk2)
        self.fkb = QPushButton("计算并显示"); self.fkb.clicked.connect(self._fk); fl.addRow("", self.fkb)
        self.fkr = QLabel("x: —  y: —"); fl.addRow("", self.fkr)
        self.fks = QDoubleSpinBox(); self.fks.setRange(1, 720); self.fks.setValue(90); self.fks.setSuffix(" °/s")
        fl.addRow("速度:", self.fks)
        self.fkm = QPushButton("发送"); self.fkm.clicked.connect(self._send); fl.addRow("", self.fkm)
        ll.addWidget(fg)

        # 逆运动学
        ig = QGroupBox("逆运动学")
        il = QFormLayout(ig); il.setSpacing(6)
        self.ikx = QDoubleSpinBox(); self.ikx.setRange(-300, 300); self.ikx.setSuffix(" mm"); self.ikx.setDecimals(1)
        self.ikx.setValue(0)
        il.addRow("X:", self.ikx)
        self.iky = QDoubleSpinBox(); self.iky.setRange(-300, 300); self.iky.setSuffix(" mm"); self.iky.setDecimals(1)
        self.iky.setValue(-200)
        il.addRow("Y:", self.iky)
        self.ikb = QPushButton("计算并显示"); self.ikb.clicked.connect(self._ik); il.addRow("", self.ikb)
        self.ikr = QLabel("θ₁: —  θ₂: —"); il.addRow("", self.ikr)
        ll.addWidget(ig)
        ll.addStretch()

        # ---- 主区 ----
        right = QWidget(); rl = QVBoxLayout(right); rl.setSpacing(6)
        self.canvas = Canvas(); rl.addWidget(self.canvas, 1)

        lf = QFrame(); lf.setStyleSheet("QFrame{background:#252538;border:1px solid #45475a;border-radius:6px;}")
        llf = QVBoxLayout(lf); llf.setContentsMargins(6, 6, 6, 6)
        llf.addWidget(QLabel("日志"))
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setMaximumHeight(140)
        self.log.setStyleSheet("background:#1e1e2e;color:#cdd6f4;border:none;font-family:Consolas,monospace;font-size:11px;")
        llf.addWidget(self.log)
        cr = QHBoxLayout()
        self.cmd_i = QLineEdit(); self.cmd_i.setPlaceholderText("原始指令...")
        self.cmd_i.returnPressed.connect(self._raw); cr.addWidget(self.cmd_i)
        self.cmd_b = QPushButton("发送"); self.cmd_b.clicked.connect(self._raw); cr.addWidget(self.cmd_b)
        llf.addLayout(cr)
        rl.addWidget(lf)

        ml.addWidget(left); ml.addWidget(right, 1)
        self.statusBar = QStatusBar(); self.setStatusBar(self.statusBar); self.statusBar.showMessage("就绪")

    # ====== 串口 ======
    def _ref(self):
        self.port.clear()
        for p in list_ports(): self.port.addItem(p)
        if self.port.count() == 0: self.port.addItem("无端口")

    def _tog(self):
        if self.serial.connected:
            self.serial.disconnect(); self.cb.setText("连接")
        else:
            p = self.port.currentText()
            if not p or p == "无端口": return
            self.serial.connect(p, int(self.baud.currentText()))

    def _on_status(self, ok, msg):
        self.cb.setText("断开" if ok else "连接")
        self.st.setText("● 已连接" if ok else "● 未连接")
        self.st.setStyleSheet(f"color:#{'a6e3a1' if ok else '6c7086'}")
        self._log(f"系统: {msg}")
        if ok:
            QTimer.singleShot(100, lambda: self._cmd("E 1"))
            QTimer.singleShot(200, lambda: self._cmd("V 2000"))

    def _on_data(self, data: bytes):
        try:
            t = data.decode("ascii", errors="replace")
            if not t.strip(): return
            self._rx += t
            while "\n" in self._rx:
                i = self._rx.index("\n")
                l = self._rx[:i].strip("\r").strip()
                self._rx = self._rx[i + 1:]
                if l: self._log(f"← {l}")
        except Exception:
            pass

    def _log(self, m):
        from datetime import datetime
        self.log.append(f"[{datetime.now().strftime('%H:%M:%S')}] {m}")
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def _cmd(self, c: str):
        if self.serial.connected:
            self._log(f"→ {c}")
            self.serial.send((c + "\r\n").encode())

    def _raw(self):
        t = self.cmd_i.text().strip()
        if t and self.serial.connected:
            self.cmd_i.clear(); self._cmd(t)

    # ====== 正运动学 ======
    def _fk(self):
        t1 = self.fk1.value(); t2 = self.fk2.value()
        ee = forward_kinematics(t1, t2)
        if ee:
            x, y = ee
            self.fkr.setText(f"x: {x:.2f}  y: {y:.2f}")
            self.canvas.set_angles(t1, t2)
            self._log(f"FK: θ₁={t1:.1f}° θ₂={t2:.1f}° → ({x:.2f}, {y:.2f})")
        else:
            self.fkr.setText("不可达")
            self._log(f"FK: θ₁={t1:.1f}° θ₂={t2:.1f}° → 不可达")

    def _send(self):
        if not self.serial.connected: self._log("未连接"); return
        t1 = int(self.fk1.value()); t2 = int(self.fk2.value()); spd = int(self.fks.value())
        self._cmd(f"M {t1} {t2} {spd}")

    # ====== 逆运动学 ======
    def _ik(self):
        x = self.ikx.value(); y = self.iky.value()
        ik = inverse_kinematics(x, y)
        if ik:
            t1, t2 = ik
            self.ikr.setText(f"θ₁: {t1:.2f}°  θ₂: {t2:.2f}°")
            self.canvas.set_angles(t1, t2)
            self._log(f"IK: ({x:.2f}, {y:.2f}) → θ₁={t1:.1f}° θ₂={t2:.1f}°")
            self.fk1.setValue(round(t1, 1))
            self.fk2.setValue(round(t2, 1))
            ee = forward_kinematics(t1, t2)
            if ee:
                self.fkr.setText(f"x: {ee[0]:.2f}  y: {ee[1]:.2f}")
        else:
            self.ikr.setText("不可达")
            self._log(f"IK: ({x:.2f}, {y:.2f}) → 不可达")

    def closeEvent(self, e):
        if self.serial.connected: self.serial.disconnect()
        e.accept()
