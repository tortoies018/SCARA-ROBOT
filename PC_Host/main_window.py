"""
SCARA 控制主窗口
包含 Canvas (机械臂渲染) 和 MainW (主窗口)
"""
import math
from PyQt6.QtCore import QTimer, Qt, QPointF
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QComboBox, QLabel, QTextEdit, QLineEdit,
    QFormLayout, QDoubleSpinBox, QStatusBar, QFrame, QGridLayout
)
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont
from serial_worker import SerialWorker, list_ports

# 机械臂长度参数
ARM1 = 150.0
ARM2 = 100.0


def forward_kinematics(t1: float, t2: float) -> tuple[float, float]:
    """正运动学: 关节角度 → 末端笛卡尔坐标"""
    r1, r2 = math.radians(t1), math.radians(t2)
    x = ARM1 * math.cos(r1) + ARM2 * math.cos(r1 + r2)
    y = ARM1 * math.sin(r1) + ARM2 * math.sin(r1 + r2)
    return x, y


class Canvas(QWidget):
    """SCARA 机械臂 2D 画布"""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(450, 350)
        self.setMouseTracking(True)
        self.t1 = 0.0       # 关节1角度
        self.t2 = 0.0       # 关节2角度
        self.scale = 1.5    # 缩放系数

    def set_angles(self, t1: float, t2: float):
        """更新关节角度并重绘"""
        self.t1 = t1
        self.t2 = t2
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2

        # 背景 + 网格
        p.fillRect(self.rect(), QColor("#1e1e2e"))
        p.setPen(QPen(QColor("#313244"), 1))
        for x in range(int(cx % 30), w, 30):
            p.drawLine(x, 0, x, h)
        for y in range(int(cy % 30), h, 30):
            p.drawLine(0, y, w, y)

        # 坐标轴
        p.setPen(QPen(QColor("#45475a"), 1))
        p.drawLine(0, int(cy), w, int(cy))
        p.drawLine(int(cx), 0, int(cx), h)

        # 绘制机械臂 (坐标系变换)
        p.save()
        p.translate(cx, cy)
        p.scale(1, -1)
        p.scale(self.scale, self.scale)
        ss = self.scale

        r1 = math.radians(self.t1)
        r2 = math.radians(self.t2)
        jx = ARM1 * math.cos(r1)          # 关节1坐标
        jy = ARM1 * math.sin(r1)
        ex = ARM1 * math.cos(r1) + ARM2 * math.cos(r1 + r2)  # 末端坐标
        ey = ARM1 * math.sin(r1) + ARM2 * math.sin(r1 + r2)

        # 大臂 (红色)
        p.setPen(QPen(QColor("#f38ba8"), 5 / ss))
        p.drawLine(QPointF(0, 0), QPointF(jx, jy))
        # 小臂 (蓝色)
        p.setPen(QPen(QColor("#89b4fa"), 5 / ss))
        p.drawLine(QPointF(jx, jy), QPointF(ex, ey))

        # 关节圆圈
        p.setBrush(QBrush(QColor("#f38ba8")))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(0, 0), 5 / ss, 5 / ss)    # 基座
        p.setBrush(QBrush(QColor("#a6e3a1")))
        p.drawEllipse(QPointF(jx, jy), 4 / ss, 4 / ss)   # 关节1
        p.setBrush(QBrush(QColor("#89b4fa")))
        p.drawEllipse(QPointF(ex, ey), 4.5 / ss, 4.5 / ss)  # 末端

        p.restore()

        # 信息栏
        p.setPen(QColor("#a6adc8"))
        p.setFont(QFont("Segoe UI", 11))
        p.drawText(
            12, 24,
            f"θ₁={self.t1:.1f}°  θ₂={self.t2:.1f}°  "
            f"EE=({ex:.1f},{ey:.1f})  [{self.scale:.1f}x]"
        )

    def wheelEvent(self, event):
        """滚轮缩放"""
        f = 1.08 if event.angleDelta().y() > 0 else 1 / 1.08
        self.scale = max(0.2, min(20, self.scale * f))
        self.update()


class MainW(QMainWindow):
    """SCARA 控制主窗口"""

    def __init__(self):
        super().__init__()
        self.serial = SerialWorker()
        self._rx = ""  # 串口接收缓冲区
        self._build_ui()
        self.serial.signals.data_received.connect(self._on_data)
        self.serial.signals.connection_status.connect(self._on_status)
        self.setWindowTitle("SCARA 控制")
        self.resize(900, 650)

    def _build_ui(self):
        """构建 UI 布局"""
        cw = QWidget()
        self.setCentralWidget(cw)
        ml = QHBoxLayout(cw)
        ml.setContentsMargins(8, 8, 8, 8)

        # ========== 左侧面板 ==========
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setSpacing(6)

        # 串口连接组
        sg = QGroupBox("串口")
        sl = QGridLayout(sg)
        sl.addWidget(QLabel("端口:"), 0, 0)
        self.port = QComboBox()
        sl.addWidget(self.port, 0, 1)
        self.rf = QPushButton("刷新")
        self.rf.clicked.connect(self._refresh)
        sl.addWidget(self.rf, 0, 2)
        sl.addWidget(QLabel("波特率:"), 1, 0)
        self.baud = QComboBox()
        self.baud.addItems(["9600", "19200", "38400", "57600", "115200", "230400"])
        self.baud.setCurrentText("115200")
        sl.addWidget(self.baud, 1, 1)
        self.cb = QPushButton("连接")
        self.cb.clicked.connect(self._toggle)
        sl.addWidget(self.cb, 1, 2)
        self.st = QLabel("● 未连接")
        sl.addWidget(self.st, 2, 0, 1, 3)
        ll.addWidget(sg)

        # 正运动学组
        fg = QGroupBox("正运动学")
        fl = QFormLayout(fg)
        fl.setSpacing(6)
        self.fk1 = QDoubleSpinBox()
        self.fk1.setRange(-180, 180)
        self.fk1.setSuffix("°")
        self.fk1.setDecimals(1)
        fl.addRow("θ₁:", self.fk1)
        self.fk2 = QDoubleSpinBox()
        self.fk2.setRange(-180, 180)
        self.fk2.setSuffix("°")
        self.fk2.setDecimals(1)
        fl.addRow("θ₂:", self.fk2)
        self.fkb = QPushButton("计算")
        self.fkb.clicked.connect(self._fk)
        fl.addRow("", self.fkb)
        self.fkr = QLabel("x: —  y: —")
        fl.addRow("", self.fkr)
        self.fks = QDoubleSpinBox()
        self.fks.setRange(1, 720)
        self.fks.setValue(90)
        self.fks.setSuffix(" °/s")
        fl.addRow("速度:", self.fks)
        self.fkm = QPushButton("发送")
        self.fkm.clicked.connect(self._send)
        fl.addRow("", self.fkm)
        ll.addWidget(fg)

        # 电机使能控制
        eg = QGroupBox("电机")
        el = QHBoxLayout(eg)
        self.en_btn = QPushButton("使能")
        self.en_btn.clicked.connect(lambda: self._do_enable(1))
        self.dis_btn = QPushButton("禁能")
        self.dis_btn.clicked.connect(lambda: self._do_enable(0))
        el.addWidget(self.en_btn)
        el.addWidget(self.dis_btn)
        ll.addWidget(eg)
        ll.addStretch()

        # ========== 右侧面板 ==========
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setSpacing(6)

        # 画布
        self.canvas = Canvas()
        rl.addWidget(self.canvas, 1)

        # 日志区域
        lf = QFrame()
        lf.setStyleSheet(
            "QFrame{background:#252538;border:1px solid #45475a;border-radius:6px;}"
        )
        llf = QVBoxLayout(lf)
        llf.setContentsMargins(6, 6, 6, 6)
        llf.addWidget(QLabel("日志"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(150)
        self.log.setStyleSheet(
            "background:#1e1e2e;color:#cdd6f4;border:none;"
            "font-family:Consolas,monospace;font-size:11px;"
        )
        llf.addWidget(self.log)

        # 原始指令输入
        cmd_row = QHBoxLayout()
        self.cmd_i = QLineEdit()
        self.cmd_i.setPlaceholderText("原始指令...")
        self.cmd_i.returnPressed.connect(self._raw)
        cmd_row.addWidget(self.cmd_i)
        self.cmd_b = QPushButton("发送")
        self.cmd_b.clicked.connect(self._raw)
        cmd_row.addWidget(self.cmd_b)
        llf.addLayout(cmd_row)
        rl.addWidget(lf)

        ml.addWidget(left)
        ml.addWidget(right, 1)

        # 状态栏
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("就绪")

    # ==================== 串口操作 ====================
    def _refresh(self):
        """刷新端口列表"""
        self.port.clear()
        for p in list_ports():
            self.port.addItem(p)
        if self.port.count() == 0:
            self.port.addItem("无端口")

    def _toggle(self):
        """连接/断开"""
        if self.serial.connected:
            self.serial.disconnect()
            self.cb.setText("连接")
        else:
            p = self.port.currentText()
            if not p or p == "无端口":
                return
            self.serial.connect(p, int(self.baud.currentText()))

    def _on_status(self, ok: bool, msg: str):
        """串口状态变化"""
        self.cb.setText("断开" if ok else "连接")
        self.st.setText("● 已连接" if ok else "● 未连接")
        self.st.setStyleSheet(f"color:#{'a6e3a1' if ok else '6c7086'}")
        self._log(f"系统: {msg}")
        if ok:
            QTimer.singleShot(100, lambda: self._do_enable(1))
            QTimer.singleShot(200, lambda: self.serial.send(b"V 2000\r\n"))

    def _on_data(self, data: bytes):
        """串口数据接收"""
        try:
            t = data.decode("ascii", errors="replace")
            if not t.strip():
                return
            self._rx += t
            while "\n" in self._rx:
                i = self._rx.index("\n")
                line = self._rx[:i].strip("\r").strip()
                self._rx = self._rx[i + 1 :]
                if line:
                    self._log(f"← {line}")
        except Exception:
            pass

    def _log(self, m):
        """日志输出"""
        from datetime import datetime

        self.log.append(f"[{datetime.now().strftime('%H:%M:%S')}] {m}")
        self.log.verticalScrollBar().setValue(
            self.log.verticalScrollBar().maximum()
        )

    # ==================== 指令发送 ====================
    def _raw(self):
        """发送原始指令"""
        t = self.cmd_i.text().strip()
        if t and self.serial.connected:
            self.cmd_i.clear()
            self._log(f"→ {t}")
            self.serial.send((t + "\r\n").encode())

    # ==================== 正运动学 ====================
    def _fk(self):
        """计算正运动学"""
        t1 = self.fk1.value()
        t2 = self.fk2.value()
        x, y = forward_kinematics(t1, t2)
        self.fkr.setText(f"x: {x:.2f}  y: {y:.2f}")
        self.canvas.set_angles(t1, t2)
        self._log(f"FK: θ₁={t1:.1f}° θ₂={t2:.1f}° → ({x:.2f}, {y:.2f})")

    def _send(self):
        """发送角度指令到机器人: M θ1 θ2 speed"""
        if not self.serial.connected:
            self._log("未连接")
            return
        t1 = int(self.fk1.value())
        t2 = int(self.fk2.value())
        spd = int(self.fks.value())
        cmd = f"M {t1} {t2} {spd}"
        self.serial.send((cmd + "\r\n").encode())
        self._log(f"→ {cmd}")

    def _do_enable(self, on: int):
        """电机使能/禁能"""
        if not self.serial.connected:
            self._log("未连接")
            return
        cmd = f"E {on}"
        self.serial.send((cmd + "\r\n").encode())
        self._log(f"→ {cmd}")

    def closeEvent(self, event):
        """窗口关闭时断开串口"""
        if self.serial.connected:
            self.serial.disconnect()
        event.accept()
