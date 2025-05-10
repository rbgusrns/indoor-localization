from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QPixmap, QPen
from PyQt5.QtCore import Qt, QTimer
import sys
import math
import serial
import threading

class MapViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Tracking Map")
        self.map = QPixmap("105.png")
        self.resize(self.map.width(), self.map.height())

        self.path = []
        self.scale = 74  # 1m = 74px
        self.current_heading = 0  # deg

    def update_position(self, x_m, y_m, heading_deg):
        x_px = int(x_m * self.scale)
        y_px = int(y_m * self.scale)
        self.path.append((x_px, y_px))
        self.current_heading = heading_deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self.map)

        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        for p in self.path:
            painter.drawPoint(*p)

        if self.path:
            x, y = self.path[-1]
            self.draw_arrow(painter, x, y, self.current_heading)

    def draw_arrow(self, painter, x, y, angle_deg):
        length = 20
        rad = math.radians(angle_deg)
        dx = length * math.cos(rad)
        dy = length * math.sin(rad)

        pen = QPen(Qt.blue, 4)
        painter.setPen(pen)
        painter.drawLine(x, y, int(x + dx), int(y + dy))


# 🔌 시리얼 포트 설정 (ESP8266 → Raspberry Pi로 POS,x,y 형식으로 전송한다고 가정)
SERIAL_PORT = '/dev/ttyUSB0'  # 혹은 /dev/ttyS0, COM3 등
BAUD_RATE = 9600

def serial_thread_func(viewer):
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith("POS"):
            try:
                _, x_str, y_str, h_str = line.split(",")
                x = float(x_str)
                y = float(y_str)
                heading = float(h_str)
                # GUI 업데이트는 main thread에서
                QTimer.singleShot(0, lambda: viewer.update_position(x, y, heading))
            except Exception as e:
                print("Parse error:", e)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = MapViewer()
    viewer.show()

    # 시리얼 쓰레드 시작
    t = threading.Thread(target=serial_thread_func, args=(viewer,), daemon=True)
    t.start()

    sys.exit(app.exec_())
