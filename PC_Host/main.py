"""SCARA 机器人上位机控制软件入口"""
import sys
from PyQt6.QtWidgets import QApplication
from main_window import MainW


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainW()
    w.show()
    w._refresh()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
