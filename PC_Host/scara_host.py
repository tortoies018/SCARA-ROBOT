#!/usr/bin/env python3
import sys, os, math, time
from typing import Optional
import serial
import serial.tools.list_ports
from threading import Thread, Lock
from PyQt6.QtCore import QTimer, Qt, QPointF, pyqtSignal, QObject
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QComboBox, QLabel, QTextEdit, QLineEdit,
    QFormLayout, QDoubleSpinBox, QStatusBar, QFrame, QGridLayout
)
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPaintEvent

ARM1 = 150.0; ARM2 = 100.0

def fk(t1: float, t2: float) -> tuple[float, float]:
    r1, r2 = math.radians(t1), math.radians(t2)
    return (ARM1 * math.cos(r1) + ARM2 * math.cos(r1 + r2),
            ARM1 * math.sin(r1) + ARM2 * math.sin(r1 + r2))

class SerialSignals(QObject):
    data_received = pyqtSignal(bytes)
    connection_status = pyqtSignal(bool, str)

class SerialWorker:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.signals = SerialSignals()
        self._th: Optional[Thread] = None
        self._run_flag = False
        self.connected = False
        self._lock = Lock()

    def connect(self, port: str, baud: int = 115200) -> bool:
        try:
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = serial.Serial(port, baud, timeout=0.5)
            if self.ser.is_open:
                self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
                self.connected = True; self._run_flag = True
                self._th = Thread(target=self._run, daemon=True); self._th.start()
                self.signals.connection_status.emit(True, f"已连接 {port} @ {baud} bps")
                return True
            self.signals.connection_status.emit(False, f"打开失败: {port}")
            return False
        except Exception as e:
            self.signals.connection_status.emit(False, f"连接失败: {e}")
            return False

    def disconnect(self):
        self._run_flag = False; self.connected = False
        if self._th: self._th.join(timeout=1)
        with self._lock:
            if self.ser and self.ser.is_open: self.ser.close()
            self.ser = None
        self.signals.connection_status.emit(False, "已断开")

    def send(self, data: bytes) -> bool:
        with self._lock:
            if self.ser and self.ser.is_open:
                try: self.ser.write(data); return True
                except: pass
        return False

    def _run(self):
        data = b""
        while self._run_flag and self.connected:
            try:
                with self._lock:
                    if self.ser and self.ser.is_open and self.ser.in_waiting:
                        data = self.ser.read(self.ser.in_waiting)
                if data:
                    self.signals.data_received.emit(data)
                    data = b""
                time.sleep(0.01)
            except:
                self._run_flag = False; self.connected = False
                self.signals.connection_status.emit(False, "串口错误")

def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

class Canvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(450, 350)
        self.setMouseTracking(True)
        self.t1 = 0.0; self.t2 = 0.0; self.scale = 1.5

    def set_angles(self, t1: float, t2: float):
        self.t1 = t1; self.t2 = t2; self.update()

    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height(); cx, cy = w/2, h/2
        p.fillRect(self.rect(), QColor("#1e1e2e"))
        p.setPen(QPen(QColor("#313244"), 1))
        for x in range(int(cx % 30), w, 30): p.drawLine(x, 0, x, h)
        for y in range(int(cy % 30), h, 30): p.drawLine(0, y, w, y)
        p.setPen(QPen(QColor("#45475a"), 1))
        p.drawLine(0, int(cy), w, int(cy)); p.drawLine(int(cx), 0, int(cx), h)
        p.save()
        p.translate(cx, cy); p.scale(1, -1); p.scale(self.scale, self.scale)
        ss = self.scale
        r1 = math.radians(self.t1); r2 = math.radians(self.t2)
        jx = ARM1 * math.cos(r1); jy = ARM1 * math.sin(r1)
        ex = ARM1 * math.cos(r1) + ARM2 * math.cos(r1 + r2)
        ey = ARM1 * math.sin(r1) + ARM2 * math.sin(r1 + r2)
        p.setPen(QPen(QColor("#f38ba8"), 5/ss)); p.drawLine(QPointF(0,0), QPointF(jx,jy))
        p.setPen(QPen(QColor("#89b4fa"), 5/ss)); p.drawLine(QPointF(jx,jy), QPointF(ex,ey))
        p.setBrush(QBrush(QColor("#f38ba8"))); p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(0,0), 5/ss, 5/ss)
        p.setBrush(QBrush(QColor("#a6e3a1"))); p.drawEllipse(QPointF(jx,jy), 4/ss, 4/ss)
        p.setBrush(QBrush(QColor("#89b4fa"))); p.drawEllipse(QPointF(ex,ey), 4.5/ss, 4.5/ss)
        p.restore()
        p.setPen(QColor("#a6adc8")); p.setFont(QFont("Segoe UI", 11))
        p.drawText(12, 24, f"θ₁={self.t1:.1f}°  θ₂={self.t2:.1f}°  EE=({ex:.1f},{ey:.1f})  [{self.scale:.1f}x]")

    def wheelEvent(self, e):
        f = 1.08 if e.angleDelta().y() > 0 else 1/1.08
        self.scale = max(0.2, min(20, self.scale * f)); self.update()

class MainW(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial = SerialWorker(); self._rx = ""
        self._build()
        self.serial.signals.data_received.connect(self._on_data)
        self.serial.signals.connection_status.connect(self._on_st)
        self.setWindowTitle("SCARA 控制"); self.resize(900, 650)

    def _build(self):
        c = QWidget(); self.setCentralWidget(c)
        ml = QHBoxLayout(c); ml.setContentsMargins(8,8,8,8)
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

        fg = QGroupBox("正运动学")
        fl = QFormLayout(fg); fl.setSpacing(6)
        self.fk1 = QDoubleSpinBox(); self.fk1.setRange(-180,180); self.fk1.setSuffix("°"); self.fk1.setDecimals(1)
        fl.addRow("θ₁:", self.fk1)
        self.fk2 = QDoubleSpinBox(); self.fk2.setRange(-180,180); self.fk2.setSuffix("°"); self.fk2.setDecimals(1)
        fl.addRow("θ₂:", self.fk2)
        self.fkb = QPushButton("计算"); self.fkb.clicked.connect(self._fk); fl.addRow("", self.fkb)
        self.fkr = QLabel("x: —  y: —"); fl.addRow("", self.fkr)
        self.fks = QDoubleSpinBox(); self.fks.setRange(1,720); self.fks.setValue(90); self.fks.setSuffix(" °/s")
        fl.addRow("速度:", self.fks)
        self.fkm = QPushButton("发送"); self.fkm.clicked.connect(self._send); fl.addRow("", self.fkm)
        ll.addWidget(fg); ll.addStretch()

        r = QWidget(); rl = QVBoxLayout(r); rl.setSpacing(6)
        self.canvas = Canvas(); rl.addWidget(self.canvas, 1)
        lf = QFrame(); lf.setStyleSheet("QFrame{background:#252538;border:1px solid #45475a;border-radius:6px;}")
        llf = QVBoxLayout(lf); llf.setContentsMargins(6,6,6,6)
        llf.addWidget(QLabel("日志"))
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setMaximumHeight(150)
        self.log.setStyleSheet("background:#1e1e2e;color:#cdd6f4;border:none;font-family:Consolas;font-size:11px;")
        llf.addWidget(self.log)
        cmd_row = QHBoxLayout()
        self.cmd_i = QLineEdit(); self.cmd_i.setPlaceholderText("原始指令...")
        self.cmd_i.returnPressed.connect(self._raw); cmd_row.addWidget(self.cmd_i)
        self.cmd_b = QPushButton("发送"); self.cmd_b.clicked.connect(self._raw); cmd_row.addWidget(self.cmd_b)
        llf.addLayout(cmd_row)
        rl.addWidget(lf)
        ml.addWidget(left); ml.addWidget(r, 1)
        self.statusBar = QStatusBar(); self.setStatusBar(self.statusBar); self.statusBar.showMessage("就绪")

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

    def _on_st(self, ok, msg):
        self.cb.setText("断开" if ok else "连接")
        self.st.setText("● 已连接" if ok else "● 未连接")
        self.st.setStyleSheet(f"color:#{'a6e3a1' if ok else '6c7086'}")
        self._lg(f"系统: {msg}")
        if ok:
            QTimer.singleShot(100, lambda: self.serial.send(b"V 90 90\r\n"))

    def _on_data(self, data: bytes):
        try:
            t = data.decode('ascii', errors='replace')
            if not t.strip(): return
            self._rx += t
            while '\n' in self._rx:
                i = self._rx.index('\n')
                l = self._rx[:i].strip('\r').strip()
                self._rx = self._rx[i+1:]
                if l: self._lg(f"← {l}")
        except: pass

    def _lg(self, m):
        from datetime import datetime
        self.log.append(f"[{datetime.now().strftime('%H:%M:%S')}] {m}")
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def _raw(self):
        t = self.cmd_i.text().strip()
        if t and self.serial.connected:
            self.cmd_i.clear(); self._lg(f"→ {t}")
            self.serial.send((t + '\r\n').encode())

    def _fk(self):
        t1 = self.fk1.value(); t2 = self.fk2.value()
        x, y = fk(t1, t2)
        self.fkr.setText(f"x: {x:.2f}  y: {y:.2f}")
        self.canvas.set_angles(t1, t2)
        self._lg(f"FK: θ₁={t1:.1f}° θ₂={t2:.1f}° → ({x:.2f}, {y:.2f})")

    def _send(self):
        if not self.serial.connected: self._lg("未连接"); return
        t1 = int(self.fk1.value()); t2 = int(self.fk2.value()); spd = int(self.fks.value())
        cmd = f"M {t1} {t2} {spd}"
        self.serial.send((cmd + '\r\n').encode())
        self._lg(f"→ {cmd}")

    def closeEvent(self, e):
        if self.serial.connected: self.serial.disconnect()
        e.accept()

def main():
    app = QApplication(sys.argv); app.setStyle("Fusion")
    w = MainW(); w.show(); w._ref()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
