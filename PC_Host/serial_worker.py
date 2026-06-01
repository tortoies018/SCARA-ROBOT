"""
串口工作线程模块
基于 threading.Thread 异步接收，pyqtSignal 向主线程发送数据
"""
import serial
import serial.tools.list_ports
from threading import Thread, Lock
from PyQt6.QtCore import pyqtSignal, QObject


class SerialSignals(QObject):
    """跨线程信号容器"""
    data_received = pyqtSignal(bytes)
    connection_status = pyqtSignal(bool, str)


class SerialWorker:
    """串口异步收发器"""

    def __init__(self):
        self.ser: serial.Serial | None = None
        self.signals = SerialSignals()
        self._th: Thread | None = None
        self._run_flag = False
        self.connected = False
        self._lock = Lock()

    def connect(self, port: str, baud: int = 115200) -> bool:
        """打开串口并启动接收线程"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, baud, timeout=0.5)
            if self.ser.is_open:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.connected = True
                self._run_flag = True
                self._th = Thread(target=self._run, daemon=True)
                self._th.start()
                self.signals.connection_status.emit(True, f"已连接 {port} @ {baud} bps")
                return True
            self.signals.connection_status.emit(False, f"打开失败: {port}")
            return False
        except Exception as e:
            self.signals.connection_status.emit(False, f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开串口"""
        self._run_flag = False
        self.connected = False
        if self._th:
            self._th.join(timeout=1)
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None
        self.signals.connection_status.emit(False, "已断开")

    def send(self, data: bytes) -> bool:
        """发送数据 (线程安全)"""
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(data)
                    return True
                except Exception:
                    pass
        return False

    def _run(self):
        """接收线程: 轮询串口读取数据 → 信号发射"""
        data = b""
        while self._run_flag and self.connected:
            try:
                with self._lock:
                    if self.ser and self.ser.is_open and self.ser.in_waiting:
                        data = self.ser.read(self.ser.in_waiting)
                if data:
                    self.signals.data_received.emit(data)
                    data = b""
            except Exception:
                self._run_flag = False
                self.connected = False
                self.signals.connection_status.emit(False, "串口错误")
            import time
            time.sleep(0.01)


def list_ports():
    """列出可用串口"""
    return [p.device for p in serial.tools.list_ports.comports()]
