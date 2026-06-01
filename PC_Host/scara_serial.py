import serial
import serial.tools.list_ports
from threading import Thread, Lock, Event
import time


class SCARASerial:
    def __init__(self):
        self.ser: serial.Serial | None = None
        self.lock = Lock()
        self._response = ""
        self._response_ready = Event()
        self._reader_thread: Thread | None = None
        self._running = False
        self.connected = False
        self._buffer = ""

    def list_ports(self) -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port: str, baud: int = 115200) -> bool:
        if self.connected:
            self.disconnect()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self._running = True
            self._buffer = ""
            self._reader_thread = Thread(target=self._reader, daemon=True)
            self._reader_thread.start()
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            return True
        except Exception as e:
            print(f"Connect error: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1)
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        self.connected = False
        self._response_ready.set()

    def send(self, cmd: str) -> str:
        if not self.connected or not self.ser:
            return "ER NOT CONNECTED"
        self._response = ""
        self._response_ready.clear()
        self._buffer = ""
        try:
            with self.lock:
                self.ser.write(cmd.encode())
            self._response_ready.wait(timeout=5.0)
            return self._response if self._response else "ER TIMEOUT"
        except Exception as e:
            return f"ER {e}"

    def send_async(self, cmd: str):
        if not self.connected or not self.ser:
            return
        try:
            with self.lock:
                self.ser.write(cmd.encode())
        except Exception:
            pass

    def _reader(self):
        while self._running:
            try:
                with self.lock:
                    if self.ser and self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace')
                        self._buffer += data
            except Exception:
                pass
            while '\n' in self._buffer:
                idx = self._buffer.index('\n')
                line = self._buffer[:idx].strip()
                self._buffer = self._buffer[idx + 1:]
                if not self._response_ready.is_set():
                    self._response = line
                    self._response_ready.set()
            time.sleep(0.01)

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            resp = self.send("Q\r\n")
            if "RDY" in resp:
                return True
            time.sleep(0.05)
        return False

    def move_relative(self, d1: int, d2: int, speed: int = 2000) -> str:
        return self.send(f"M {d1} {d2} {speed}\r\n")

    def move_absolute(self, s1: int, s2: int, speed: int = 2000) -> str:
        return self.send(f"A {s1} {s2} {speed}\r\n")

    def home(self) -> str:
        return self.send("H\r\n")

    def pen_up(self) -> str:
        return self.send("P0\r\n")

    def pen_down(self) -> str:
        return self.send("P1\r\n")

    def stop(self) -> str:
        return self.send("!\r\n")

    def query(self) -> str:
        return self.send("Q\r\n")

    def set_speed(self, speed: int) -> str:
        return self.send(f"V {speed}\r\n")

    def parse_position(self, resp: str) -> tuple[int, int, str] | None:
        if resp.startswith("POS"):
            parts = resp.split()
            if len(parts) >= 3:
                try:
                    s1 = int(parts[1])
                    s2 = int(parts[2])
                    status = parts[3] if len(parts) > 3 else ""
                    return s1, s2, status
                except ValueError:
                    pass
        return None
