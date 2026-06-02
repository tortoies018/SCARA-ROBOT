"""
SCARA 并联机器人控制主窗口
双摇杆五连杆结构: 主动臂110mm, 从动臂220mm, 电机间距160mm
"""
import math
import time
from PyQt6.QtCore import QTimer, Qt, QPointF
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QComboBox, QLabel, QTextEdit, QLineEdit,
    QFormLayout, QDoubleSpinBox, QStatusBar, QFrame, QGridLayout,
    QScrollArea, QProgressBar
)
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QMouseEvent
from serial_worker import SerialWorker, list_ports

# ======================== 机械参数 ========================
L1 = 110.0          # 主动臂长 mm
L2 = 220.0          # 从动臂长 mm
D  = 160.0          # 电机间距 mm
HALF_D = D / 2

def norm_angle(a: float) -> float:
    """归一到 (-π, π]"""
    while a > math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def forward_kinematics(deg1: float, deg2: float) -> tuple[float, float] | None:
    r1, r2 = math.radians(deg1), math.radians(deg2)
    p1x = -HALF_D + L1 * math.cos(r1); p1y = L1 * math.sin(r1)
    p2x =  HALF_D + L1 * math.cos(r2); p2y = L1 * math.sin(r2)
    dx = p2x - p1x; dy = p2y - p1y; d2 = dx*dx + dy*dy
    if d2 > (2*L2)**2 or d2 < 0.001: return None
    d = math.sqrt(d2); a = d/2; h2 = L2*L2 - a*a
    if h2 < 0: return None
    h = math.sqrt(h2); mx = (p1x+p2x)/2; my = (p1y+p2y)/2
    ex = mx - h * dy / d; ey = my + h * dx / d  # 上交点 (机构向上翻)
    return ex, ey

def inverse_kinematics(x: float, y: float) -> tuple[float, float] | None:
    """逆运动学: 末端坐标 → 两主动臂角度 (双解+优选 0°~180°)"""
    r1 = math.hypot(x + HALF_D, y)
    if r1 > L1+L2 + 1 or r1 < abs(L2-L1) - 1: return None
    cos_phi1 = (L1*L1 + r1*r1 - L2*L2) / (2*L1*r1)
    phi1 = math.acos(max(-1, min(1, cos_phi1)))
    a1 = math.atan2(y, x + HALF_D)
    t1_a = math.degrees(norm_angle(a1 + phi1))
    t1_b = math.degrees(norm_angle(a1 - phi1))

    r2 = math.hypot(x - HALF_D, y)
    if r2 > L1+L2 + 1 or r2 < abs(L2-L1) - 1: return None
    cos_phi2 = (L1*L1 + r2*r2 - L2*L2) / (2*L1*r2)
    phi2 = math.acos(max(-1, min(1, cos_phi2)))
    a2 = math.atan2(y, x - HALF_D)
    t2_a = math.degrees(norm_angle(a2 + phi2))
    t2_b = math.degrees(norm_angle(a2 - phi2))

    # 验证两解哪个 FK 闭环正确
    for t1, t2 in [(t1_a, t2_a), (t1_b, t2_b), (t1_a, t2_b), (t1_b, t2_a)]:
        ee = forward_kinematics(t1, t2)
        if ee is not None and math.hypot(ee[0]-x, ee[1]-y) < 1:
            return t1, t2
    return None

def interpolate_line(x1, y1, x2, y2, step_mm):
    """直线插值, 返回 (x,y) 列表"""
    dist = math.hypot(x2-x1, y2-y1)
    if dist < 0.01: return [(x1, y1)]
    n = max(1, int(dist / step_mm))
    pts = []
    for i in range(n + 1):
        t = i / n
        pts.append((x1 + (x2-x1)*t, y1 + (y2-y1)*t))
    return pts

# ======================== Canvas ========================
class Canvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(500, 400)
        self.setMouseTracking(True)
        self.t1 = 90.0; self.t2 = 90.0
        self.scale = 1.0
        self.pan = QPointF(0, 0)
        self._mouse = QPointF(0, 0)
        self._drag = False; self._drag_start = QPointF(0, 0)
        self.traj_preview: list[tuple[float, float]] = []  # 轨迹预览点

    def set_angles(self, t1, t2):
        self.t1 = t1; self.t2 = t2; self.update()

    def set_traj(self, pts: list[tuple[float, float]]):
        self.traj_preview = list(pts); self.update()

    def clear_traj(self):
        self.traj_preview.clear(); self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w/2 + self.pan.x(), h/2 + self.pan.y()

        p.fillRect(self.rect(), QColor("#1e1e2e"))

        # ---- 轨迹预览 (在变换外绘制) ----
        if self.traj_preview:
            pts = self.traj_preview
            def sx(x): return cx + x * self.scale
            def sy(y): return cy - y * self.scale
            # 路径线
            p.setPen(QPen(QColor("#fab387"), 2))
            for i in range(len(pts)-1):
                p.drawLine(QPointF(sx(pts[i][0]), sy(pts[i][1])),
                           QPointF(sx(pts[i+1][0]), sy(pts[i+1][1])))
            # 插值点
            p.setBrush(QBrush(QColor("#fab387"))); p.setPen(Qt.PenStyle.NoPen)
            for pt in pts:
                p.drawEllipse(QPointF(sx(pt[0]), sy(pt[1])), 3, 3)
            # 起点/终点高亮
            p.setBrush(QBrush(QColor("#a6e3a1")))
            p.drawEllipse(QPointF(sx(pts[0][0]), sy(pts[0][1])), 5, 5)
            p.setBrush(QBrush(QColor("#f38ba8")))
            p.drawEllipse(QPointF(sx(pts[-1][0]), sy(pts[-1][1])), 5, 5)

        # ---- 坐标变换 ----
        p.save()
        p.translate(cx, cy); p.scale(1, -1); p.scale(self.scale, self.scale)
        s = self.scale

        r1 = math.radians(self.t1); r2 = math.radians(self.t2)
        a1x = -HALF_D + L1*math.cos(r1); a1y = L1*math.sin(r1)
        a2x =  HALF_D + L1*math.cos(r2); a2y = L1*math.sin(r2)
        ee = forward_kinematics(self.t1, self.t2)
        ex, ey = ee if ee else (0, -L2)

        # mm 网格
        p.setPen(QPen(QColor("#2a2a3c"), 1/s))
        for g in range(-500, 501, 50):
            p.drawLine(QPointF(-500/s, g/s), QPointF(500/s, g/s))
            p.drawLine(QPointF(g/s, -500/s), QPointF(g/s, 500/s))

        p.setPen(QPen(QColor("#6c7086"), 2/s, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(a1x, a1y), QPointF(ex, ey))
        p.drawLine(QPointF(a2x, a2y), QPointF(ex, ey))

        p.setPen(QPen(QColor("#f38ba8"), 5/s, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(-HALF_D, 0), QPointF(a1x, a1y))
        p.setPen(QPen(QColor("#89b4fa"), 5/s, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(HALF_D, 0), QPointF(a2x, a2y))

        p.setBrush(QBrush(QColor("#45475a"))); p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(-HALF_D, 0), 6/s, 6/s)
        p.drawEllipse(QPointF(HALF_D, 0), 6/s, 6/s)
        p.setBrush(QBrush(QColor("#f38ba8")))
        p.drawEllipse(QPointF(a1x, a1y), 4.5/s, 4.5/s)
        p.setBrush(QBrush(QColor("#89b4fa")))
        p.drawEllipse(QPointF(a2x, a2y), 4.5/s, 4.5/s)
        p.setBrush(QBrush(QColor("#a6e3a1")))
        p.drawEllipse(QPointF(ex, ey), 5/s, 5/s)

        p.setPen(QPen(QColor("#fab387"), 1.5/s, Qt.PenStyle.DotLine))
        p.drawLine(QPointF(-HALF_D, 0), QPointF(HALF_D, 0))
        p.restore()

        # 文字标签 + 坐标轴刻度
        def sx(x): return cx + x*self.scale
        def sy(y): return cy - y*self.scale
        p.setFont(QFont("Segoe UI", 9))

        # X 轴刻度 (每 100mm)
        p.setPen(QColor("#585b70"))
        for v in range(-400, 401, 100):
            px, py = int(sx(v)), int(sy(0))
            if 0 < px < self.width():
                p.drawText(px-10, py+16, f"{v}")
        # Y 轴刻度
        for v in range(-400, 401, 100):
            px, py = int(sx(0)), int(sy(v))
            if 0 < py < self.height():
                p.drawText(px+8, py+4, f"{v}")
        p.drawText(int(sx(0))+8, int(sy(0))+16, "O")

        p.setFont(QFont("Segoe UI", 10))
        p.setPen(QColor("#f38ba8")); p.drawText(int(sx(-HALF_D))-14, int(sy(0))+4, "M₁")
        p.setPen(QColor("#89b4fa")); p.drawText(int(sx(HALF_D))+6, int(sy(0))+4, "M₂")
        p.setPen(QColor("#a6e3a1")); p.drawText(int(sx(ex))+8, int(sy(ey))+4, f"({ex:.0f},{ey:.0f})")
        p.setPen(QColor("#fab387")); p.drawText(int(sx(0))-14, int(sy(0))+20, f"{D}mm")

        p.setPen(QColor("#a6adc8")); p.setFont(QFont("Segoe UI", 11))
        p.drawText(14, 24,
            f"θ₁={self.t1:.1f}° θ₂={self.t2:.1f}°  "
            f"EE=({(ee[0] if ee else 0):.1f}, {(ee[1] if ee else 0):.1f})  "
            f"轨迹:{len(self.traj_preview)}点  [{self.scale:.1f}x]")

    # ====== 交互 ======
    def mousePressEvent(self, e):
        if e.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.MiddleButton):
            self._drag = True; self._drag_start = e.position()

    def mouseMoveEvent(self, e):
        if self._drag:
            self.pan += e.position() - self._drag_start
            self._drag_start = e.position(); self.update()
        else:
            w, h = self.width(), self.height()
            cx, cy = w/2 + self.pan.x(), h/2 + self.pan.y()
            self._mouse = QPointF((e.position().x()-cx)/self.scale,
                                  (cy-e.position().y())/self.scale)
            self.update()

    def mouseReleaseEvent(self, e): self._drag = False
    def wheelEvent(self, e):
        f = 1.08 if e.angleDelta().y() > 0 else 1/1.08
        self.scale = max(0.3, min(15, self.scale*f)); self.update()
    def resizeEvent(self, e): self.update(); super().resizeEvent(e)


# ======================== 主窗口 ========================
class MainW(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial = SerialWorker()
        self._rx = ""
        self._traj_queue: list[tuple[int, int, int]] = []  # (θ1, θ2, speed_hz) 队列
        self._traj_running = False
        self._build_ui()
        self.serial.signals.data_received.connect(self._on_data)
        self.serial.signals.connection_status.connect(self._on_status)
        self.setWindowTitle("SCARA 并联机器人控制")
        self.resize(1000, 720)
        self.canvas.set_angles(90, 90)

    def _build_ui(self):
        cw = QWidget(); self.setCentralWidget(cw)
        ml = QHBoxLayout(cw); ml.setContentsMargins(8, 8, 8, 8)

        # ---- 可滚动侧栏 ----
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea{border:none;background:#181825}")
        left = QWidget(); ll = QVBoxLayout(left); ll.setSpacing(6)
        ll.setContentsMargins(8, 4, 8, 8)
        scroll.setWidget(left)

        # 串口
        sg = QGroupBox("串口"); sl = QGridLayout(sg)
        sl.addWidget(QLabel("端口:"), 0, 0)
        self.port = QComboBox(); sl.addWidget(self.port, 0, 1)
        self.rf = QPushButton("刷新"); self.rf.clicked.connect(self._ref); sl.addWidget(self.rf, 0, 2)
        sl.addWidget(QLabel("波特率:"), 1, 0)
        self.baud = QComboBox(); self.baud.addItems(["9600","19200","38400","57600","115200","230400"])
        self.baud.setCurrentText("115200"); sl.addWidget(self.baud, 1, 1)
        self.cb = QPushButton("连接"); self.cb.clicked.connect(self._tog); sl.addWidget(self.cb, 1, 2)
        self.st = QLabel("● 未连接"); sl.addWidget(self.st, 2, 0, 1, 3)
        ll.addWidget(sg)

        # 电机
        eg = QGroupBox("电机"); el = QHBoxLayout(eg)
        self.en_btn = QPushButton("使能"); self.en_btn.clicked.connect(lambda: self._cmd("E 1"))
        self.dis_btn = QPushButton("禁能"); self.dis_btn.clicked.connect(lambda: self._cmd("E 0"))
        el.addWidget(self.en_btn); el.addWidget(self.dis_btn)
        ll.addWidget(eg)

        # 正运动学
        fg = QGroupBox("正运动学"); fl = QFormLayout(fg); fl.setSpacing(6)
        self.fk1 = QDoubleSpinBox(); self.fk1.setRange(-180, 180); self.fk1.setSuffix("°"); self.fk1.setDecimals(1); self.fk1.setValue(90)
        fl.addRow("θ₁:", self.fk1)
        self.fk2 = QDoubleSpinBox(); self.fk2.setRange(-180, 180); self.fk2.setSuffix("°"); self.fk2.setDecimals(1); self.fk2.setValue(90)
        fl.addRow("θ₂:", self.fk2)
        self.fkb = QPushButton("计算"); self.fkb.clicked.connect(self._fk); fl.addRow("", self.fkb)
        self.fkr = QLabel("x: —  y: —"); fl.addRow("", self.fkr)
        self.fks = QDoubleSpinBox(); self.fks.setRange(1, 720); self.fks.setValue(90); self.fks.setSuffix(" °/s")
        fl.addRow("速度:", self.fks)
        self.fkm = QPushButton("发送"); self.fkm.clicked.connect(self._send); fl.addRow("", self.fkm)
        ll.addWidget(fg)

        # 逆运动学
        ig = QGroupBox("逆运动学"); il = QFormLayout(ig); il.setSpacing(6)
        self.ikx = QDoubleSpinBox(); self.ikx.setRange(-400, 400); self.ikx.setSuffix(" mm"); self.ikx.setDecimals(1); self.ikx.setValue(0)
        il.addRow("X:", self.ikx)
        self.iky = QDoubleSpinBox(); self.iky.setRange(-400, 400); self.iky.setSuffix(" mm"); self.iky.setDecimals(1); self.iky.setValue(-200)
        il.addRow("Y:", self.iky)
        self.ikb = QPushButton("计算"); self.ikb.clicked.connect(self._ik); il.addRow("", self.ikb)
        self.ikr = QLabel("θ₁: —  θ₂: —"); il.addRow("", self.ikr)
        ll.addWidget(ig)

        # ---- 连续脉冲 ----
        pg = QGroupBox("连续脉冲"); pl = QFormLayout(pg); pl.setSpacing(4)
        self.p_spd1 = QDoubleSpinBox(); self.p_spd1.setRange(1, 720); self.p_spd1.setValue(90); self.p_spd1.setSuffix(" °/s")
        self.p_spd2 = QDoubleSpinBox(); self.p_spd2.setRange(1, 720); self.p_spd2.setValue(90); self.p_spd2.setSuffix(" °/s")
        sr1 = QHBoxLayout(); sr1.addWidget(QLabel("电机1:")); sr1.addWidget(self.p_spd1); sr1.addWidget(QLabel("电机2:")); sr1.addWidget(self.p_spd2)
        pl.addRow(sr1)
        brp = QHBoxLayout()
        self.p_fwd = QPushButton("正转"); self.p_fwd.clicked.connect(lambda: self._pulse(1, 1))
        self.p_rev = QPushButton("反转"); self.p_rev.clicked.connect(lambda: self._pulse(-1, -1))
        self.p_stop = QPushButton("停止"); self.p_stop.clicked.connect(lambda: self._cmd("C"))
        brp.addWidget(self.p_fwd); brp.addWidget(self.p_rev); brp.addWidget(self.p_stop)
        pl.addRow(brp)
        ll.addWidget(pg)

        # ---- 轨迹绘制 ----
        tg = QGroupBox("轨迹绘制")
        tl = QFormLayout(tg); tl.setSpacing(4)

        # 起点
        self.tsx = QDoubleSpinBox(); self.tsx.setRange(-400, 400); self.tsx.setDecimals(1); self.tsx.setSuffix(" mm"); self.tsx.setValue(-80)
        self.tsy = QDoubleSpinBox(); self.tsy.setRange(-400, 400); self.tsy.setDecimals(1); self.tsy.setSuffix(" mm"); self.tsy.setValue(-150)
        tr1 = QHBoxLayout(); tr1.addWidget(QLabel("起点X:")); tr1.addWidget(self.tsx); tr1.addWidget(QLabel(" Y:")); tr1.addWidget(self.tsy)
        tl.addRow(tr1)

        # 终点
        self.tex = QDoubleSpinBox(); self.tex.setRange(-400, 400); self.tex.setDecimals(1); self.tex.setSuffix(" mm"); self.tex.setValue(80)
        self.tey = QDoubleSpinBox(); self.tey.setRange(-400, 400); self.tey.setDecimals(1); self.tey.setSuffix(" mm"); self.tey.setValue(-150)
        tr2 = QHBoxLayout(); tr2.addWidget(QLabel("终点X:")); tr2.addWidget(self.tex); tr2.addWidget(QLabel(" Y:")); tr2.addWidget(self.tey)
        tl.addRow(tr2)

        # 参数
        self.tstep = QDoubleSpinBox(); self.tstep.setRange(0.5, 20); self.tstep.setValue(3); self.tstep.setSuffix(" mm")
        tr3 = QHBoxLayout(); tr3.addWidget(QLabel("步长:")); tr3.addWidget(self.tstep)
        tl.addRow(tr3)

        self.tspd = QDoubleSpinBox(); self.tspd.setRange(1, 720); self.tspd.setValue(90); self.tspd.setSuffix(" °/s")
        tr4 = QHBoxLayout(); tr4.addWidget(QLabel("速度:")); tr4.addWidget(self.tspd)
        tl.addRow(tr4)

        # 按钮
        br = QHBoxLayout()
        self.tprev = QPushButton("预览"); self.tprev.clicked.connect(self._preview_line); br.addWidget(self.tprev)
        self.tsend = QPushButton("执行"); self.tsend.clicked.connect(self._exec_line); br.addWidget(self.tsend)
        self.tclr = QPushButton("清空"); self.tclr.clicked.connect(self._clear_traj); br.addWidget(self.tclr)
        tl.addRow(br)

        # 进度
        self.tprog = QProgressBar(); self.tprog.setRange(0, 100); self.tprog.setValue(0)
        tl.addRow(self.tprog)
        self.tstat = QLabel(""); tl.addRow(self.tstat)
        ll.addWidget(tg)

        ll.addStretch()

        # ---- 主区 ----
        right = QWidget(); rl = QVBoxLayout(right); rl.setSpacing(6)
        self.canvas = Canvas(); rl.addWidget(self.canvas, 1)

        lf = QFrame(); lf.setStyleSheet("QFrame{background:#252538;border:1px solid #45475a;border-radius:6px;}")
        llf = QVBoxLayout(lf); llf.setContentsMargins(6, 6, 6, 6)
        llf.addWidget(QLabel("日志"))
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setMaximumHeight(130)
        self.log.setStyleSheet("background:#1e1e2e;color:#cdd6f4;border:none;font-family:Consolas,monospace;font-size:11px;")
        llf.addWidget(self.log)
        cr = QHBoxLayout()
        self.cmd_i = QLineEdit(); self.cmd_i.setPlaceholderText("原始指令...")
        self.cmd_i.returnPressed.connect(self._raw); cr.addWidget(self.cmd_i)
        self.cmd_b = QPushButton("发送"); self.cmd_b.clicked.connect(self._raw); cr.addWidget(self.cmd_b)
        self.clr_log = QPushButton("清空"); self.clr_log.clicked.connect(lambda: self.log.clear()); cr.addWidget(self.clr_log)
        llf.addLayout(cr)
        rl.addWidget(lf)

        ml.addWidget(scroll); ml.addWidget(right, 1)
        self.statusBar = QStatusBar(); self.setStatusBar(self.statusBar); self.statusBar.showMessage("就绪")

    # ====== 串口 ======
    def _ref(self):
        self.port.clear()
        for p in list_ports(): self.port.addItem(p)
        if self.port.count() == 0: self.port.addItem("无端口")

    def _tog(self):
        if self.serial.connected: self.serial.disconnect(); self.cb.setText("连接")
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
            QTimer.singleShot(200, lambda: self._cmd("V 90 90"))

    def _on_data(self, data: bytes):
        try:
            t = data.decode("ascii", errors="replace")
            if not t.strip(): return
            self._rx += t
            while "\n" in self._rx:
                i = self._rx.index("\n")
                l = self._rx[:i].strip("\r").strip()
                self._rx = self._rx[i+1:]
                if l:
                    self._log(f"← {l}")
                    # 轨迹队列模式下, 收到 RDY 或 OK (零位移) 后发下一点
                    # 轨迹队列: RDY(单步完成)或OK(零位移) 触发下一发
                    if self._traj_running and ("RDY" in l or l == "OK"):
                        self._send_next_traj()
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
        if t and self.serial.connected: self.cmd_i.clear(); self._cmd(t)

    # ====== 正运动学 ======
    def _fk(self):
        t1 = self.fk1.value(); t2 = self.fk2.value()
        ee = forward_kinematics(t1, t2)
        if ee:
            self.fkr.setText(f"x: {ee[0]:.2f}  y: {ee[1]:.2f}")
            self.canvas.set_angles(t1, t2)
            self._log(f"FK: {t1:.1f}° {t2:.1f}° → ({ee[0]:.1f}, {ee[1]:.1f})")
        else:
            self.fkr.setText("不可达"); self._log(f"FK: {t1:.1f}° {t2:.1f}° → 不可达")

    def _send(self):
        if not self.serial.connected: self._log("未连接"); return
        t1 = int(self.fk1.value()); t2 = int(self.fk2.value()); spd = int(self.fks.value())
        self._cmd(f"A {t1} {t2} {spd}")

    # ====== 逆运动学 ======
    def _ik(self):
        x = self.ikx.value(); y = self.iky.value()
        ik = inverse_kinematics(x, y)
        if ik:
            t1, t2 = ik
            self.ikr.setText(f"θ₁: {t1:.1f}°  θ₂: {t2:.1f}°")
            self.canvas.set_angles(t1, t2)
            self._log(f"IK: ({x:.1f},{y:.1f}) → {t1:.1f}° {t2:.1f}°")
            self.fk1.setValue(round(t1, 1)); self.fk2.setValue(round(t2, 1))
            ee = forward_kinematics(t1, t2)
            if ee: self.fkr.setText(f"x: {ee[0]:.2f}  y: {ee[1]:.2f}")
        else:
            self.ikr.setText("不可达"); self._log(f"IK: ({x:.1f},{y:.1f}) → 不可达")

    # ====== 轨迹 ======
    def _preview_line(self):
        """直线插值预览"""
        x1, y1 = self.tsx.value(), self.tsy.value()
        x2, y2 = self.tex.value(), self.tey.value()
        step = self.tstep.value()
        pts = interpolate_line(x1, y1, x2, y2, step)
        self.canvas.set_traj(pts)
        self._log(f"直线预览: ({x1:.0f},{y1:.0f})→({x2:.0f},{y2:.0f}) 步长{step}mm → {len(pts)}点")

    def _pulse(self, dir1: int, dir2: int):
        """连续脉冲: O 速度1 速度2 (符号=方向)"""
        if not self.serial.connected: self._log("未连接"); return
        s1 = int(self.p_spd1.value()) * dir1
        s2 = int(self.p_spd2.value()) * dir2
        self._cmd(f"O {s1} {s2}")

    def _clear_traj(self):
        self.canvas.clear_traj()
        self._traj_queue.clear()
        self._traj_running = False
        self.tprog.setValue(0); self.tstat.setText("")
        self._log("轨迹已清空")

    def _exec_line(self):
        """逆解并发送直线轨迹到 MCU"""
        if not self.serial.connected: self._log("未连接"); return
        if self._traj_running: self._log("正在执行中"); return

        x1, y1 = self.tsx.value(), self.tsy.value()
        x2, y2 = self.tex.value(), self.tey.value()
        step = self.tstep.value()
        speed_dps = self.tspd.value()

        # 插值
        pts = interpolate_line(x1, y1, x2, y2, step)
        self.canvas.set_traj(pts)

        # 逐个逆解, 角度连续化处理
        self._traj_queue.clear()
        prev = None
        for x, y in pts:
            ik = inverse_kinematics(x, y)
            if ik is None:
                self._log(f"  ✗ 点 ({x:.0f},{y:.0f}) 不可达, 中断")
                self._traj_queue.clear()
                return
            t1, t2 = ik
            # 角度连续化: 避免 ±360° 跳变
            if prev is not None:
                while t1 - prev[0] >  180: t1 -= 360
                while t1 - prev[0] < -180: t1 += 360
                while t2 - prev[1] >  180: t2 -= 360
                while t2 - prev[1] < -180: t2 += 360
            it1, it2 = int(t1), int(t2)
            # 跳过与上一点相同的角度
            if not self._traj_queue or (it1, it2) != (self._traj_queue[-1][0], self._traj_queue[-1][1]):
                self._traj_queue.append((it1, it2, speed_dps))
            prev = (t1, t2)

        self._log(f"轨迹: {len(self._traj_queue)}个点, 速度{speed_dps}°/s")
        self.tprog.setValue(0)
        self._traj_running = True
        self._send_next_traj()

    def _send_next_traj(self):
        """发送轨迹队列中的下一个点"""
        if not self._traj_queue:
            self._traj_running = False
            self.tprog.setValue(100)
            self.tstat.setText("✓ 完成")
            self._log("✓ 轨迹执行完成")
            return

        t1, t2, spd = self._traj_queue.pop(0)
        done = self.tprog.maximum() - len(self._traj_queue)
        total = self.tprog.maximum()
        self.tprog.setValue(int(done / total * 100) if total > 0 else 0)
        self.tstat.setText(f"发送 {done}/{total}")

        cmd = f"A {t1} {t2} {spd}"
        if self.serial.connected:
            self._log(f"→ {cmd}")
            self.serial.send((cmd + "\r\n").encode())

    def closeEvent(self, e):
        if self.serial.connected: self.serial.disconnect()
        e.accept()
