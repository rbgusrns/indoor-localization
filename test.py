import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from trilateration import EKF  # 위 EKF 클래스가 ekf_module.py에 정의돼 있다고 가정


class MapViewer(FigureCanvas):
    def __init__(self):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)

        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        self.path_x, self.path_y = [], []

    def update_position(self, x, y):
        self.path_x.append(x)
        self.path_y.append(y)

        self.ax.clear()
        self.ax.plot(self.path_x, self.path_y, 'bo-')  # 이동 경로
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        self.draw()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EKF Map Viewer")
        

        self.viewer = MapViewer()
        self.ekf = EKF(dt=0.1)

        self.vbox = QVBoxLayout()
        self.vbox.addWidget(self.viewer)

        self.btn = QPushButton("BLE 위치 업데이트")
        self.btn.clicked.connect(self.ble_update)
        self.vbox.addWidget(self.btn)

        self.setLayout(self.vbox)

        self.timer = QTimer()
        self.timer.timeout.connect(self.imu_update)
        self.timer.start(100)  # IMU는 0.1초마다 업데이트

        # 가상 입력값
        self.sim_yaw = 0.0
        self.sim_speed = 1.0
        self.t = 0.0

    def imu_update(self):
        self.sim_yaw += 2  # 도 단위
        self.ekf.predict(self.sim_yaw, self.sim_speed)
        x, y, _, _ = self.ekf.get_state()
        self.viewer.update_position(x, y)

    def ble_update(self):
        # BLE 위치는 조금 부정확하게 (노이즈 포함)
        true_x, true_y = 5 + np.random.randn()*0.5, 5 + np.random.randn()*0.5
        self.ekf.update(np.array([true_x, true_y]))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
