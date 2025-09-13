import sys
import serial
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QShortcut, QLabel
from PyQt5.QtCore import QTimer, QMutex, Qt, QFile, QTextStream, QPointF
from PyQt5.QtGui import QKeySequence

# --- 모듈 임포트 ---
from app_config import load_config
from fingerprinting import FingerprintDB
from trilateration import EKF
from ble_scanner import BLEScanThread
from map_viewer import MapViewer
from serial_reader import SerialReader
from event import SelectionDialog
from bin import create_binary_map
from Astar import find_path, create_distance_map
import socket

# --- 초기 설정 값 ---
INIT_X, INIT_Y = (0, 0)
INIT_YAW = 180.0

class IndoorPositioningApp(QWidget):
    def __init__(self, config):
        super().__init__()

        # 데이터를 보낼 대상 장치(로봇 등)의 IP 주소와 포트 번호
        self.udp_target_ip = self.config.get('udp_target_ip', "192.168.0.141") # 예시 IP
        self.udp_target_port = self.config.get('udp_target_port', 5005)
        
        # UDP 소켓 생성
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.config = config
        self.room_coords = {room['name']: (room['x'], room['y']) for room in config['rooms']}
        
        self.rssi_mutex = QMutex()
        self.rssi_data = {}
        self.current_speed = 0.0
        self.current_yaw = INIT_YAW
        self.fused_pos = (INIT_X, INIT_Y)
        self.target_room = None
        self.last_start_grid = None
        self.BLOCK_SIZE = 10

        self._init_logic_components()
        self._init_ui()
        self._connect_signals()
        self._start_timers()

    def _send_position_udp(self):
        """현재 융합된 좌표(fused_pos)를 UDP로 전송합니다."""
        # 전송할 메시지 생성 (예: "x:10.5,y:20.3")
        message = f"x:{self.fused_pos[0]:.2f},y:{self.fused_pos[1]:.2f}"
        
        # 메시지를 바이트로 인코딩하여 전송
        self.udp_socket.sendto(message.encode(), (self.udp_target_ip, self.udp_target_port))
        print(f"UDP Sent: {message}") # 디버깅용

    def _init_logic_components(self):
        self.binary_grid = create_binary_map(self.config['map_file'], block_size=self.BLOCK_SIZE)
        if self.binary_grid:
            print("이진 지도 생성 완료.")
            self.distance_map, self.max_dist = create_distance_map(self.binary_grid)
        else:
            self.close()

        self.ekf = EKF(self.config.get('ekf_dt', 1.0))
        self.fingerprint_db = FingerprintDB()
        self.fingerprint_db.load(self.config.get('fingerprint_db_path', 'fingerprint_db.json'))
        self.ble_scanner_thread = BLEScanThread(self.config)
        
        try:
            imu_port = self.config.get('imu_port', '/dev/ttyUSB0')
            baudrate = self.config.get('imu_baudrate', 115200)
            self.serial_port = serial.Serial(imu_port, baudrate)
            time.sleep(1)
            self.serial_port.write(b'ZERO\n')
            self.serial_reader = SerialReader(port=imu_port, baudrate=baudrate)
            self.serial_reader.start()
        except serial.SerialException as e:
            print(f"시리얼 포트 오류: {e}.")
            self.serial_reader = None

    def _init_ui(self):
        self.toast_label = QLabel(self)
        self.toast_label.setObjectName("Toast")
        self.toast_label.setAlignment(Qt.AlignCenter)
        self.toast_label.hide()

        self.map_viewer = MapViewer(self.config['map_file'], self.config['px_per_m_x'], self.config['px_per_m_y'])
        self.map_viewer._init_est_items(INIT_X, INIT_Y, INIT_YAW)
        self.nav_btn = QPushButton("길안내")
        self.nav_btn.setObjectName("NAV")
        self.robot_btn = QPushButton("로봇\n호출")
        self.robot_btn.setObjectName("Robot")

        right_layout = QVBoxLayout()
        right_layout.addWidget(self.nav_btn)
        right_layout.addWidget(self.robot_btn)
        main_layout = QHBoxLayout(self)
        main_layout.addWidget(self.map_viewer)
        main_layout.addLayout(right_layout)
        
        self.setWindowTitle("ODIGA")
        self.setFocusPolicy(Qt.StrongFocus)
        self.load_stylesheet('stylesheet.qss')
        self.showFullScreen()
        self.setFocus()

    def _connect_signals(self):
        self.ble_scanner_thread.detected.connect(self._on_ble_device_detected)
        if self.serial_reader:
            self.serial_reader.heading_received.connect(self._on_yaw_update)
            self.serial_reader.speed_received.connect(self._on_speed_update)
        self.nav_btn.clicked.connect(self._show_selection_dialog)
        shortcut = QShortcut(QKeySequence("G"), self)
        shortcut.activated.connect(self._start_ble_scan)

    def _start_timers(self):
        self.rssi_clear_timer = QTimer(self)
        self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache)
        self.rssi_clear_timer.start(2000)

    def _show_toast(self, message, duration=3000):
        self.toast_label.setText(message)
        self.toast_label.adjustSize()
        self._update_toast_position()
        self.toast_label.show()
        
        # --- ▼ 여기가 핵심 수정 사항 ▼ ---
        # toast_label을 다른 모든 위젯들보다 맨 위로 올립니다.
        self.toast_label.raise_()
        
        QTimer.singleShot(duration, self.toast_label.hide)

    def _update_toast_position(self):
        x = (self.width() - self.toast_label.width()) / 2
        y = 50
        self.toast_label.move(int(x), int(y))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_toast_position()

    def _update_navigation_path(self):
        if not self.target_room: return
        start_m, end_m = self.fused_pos, self.target_room
        start_grid, end_grid = self.meters_to_grid(start_m), self.meters_to_grid(end_m)
        if self.last_start_grid == start_grid: return
        self.last_start_grid = start_grid

        penalty_strength = self.config.get('penalty_strength', 10.0)
        path_grid = find_path(
            self.binary_grid, start_grid, end_grid,
            self.distance_map, self.max_dist, penalty_strength
        )
        if path_grid:
            path_pixels = [self.grid_to_pixels(pos) for pos in path_grid]
            self.map_viewer.draw_path(path_pixels)
        else:
            self.map_viewer.draw_path(None)
    
    def _on_ble_device_detected(self, rssi_vec):
        self.rssi_mutex.lock()
        self.rssi_data.update(rssi_vec)
        local_rssi_copy = self.rssi_data.copy()
        self.rssi_mutex.unlock()
        if len(local_rssi_copy) >= 6:
            pts, _, _ = self.fingerprint_db.get_position(local_rssi_copy, k=self.config['k_neighbors'])
            z = np.array([pts[0], pts[1]])
            self.ekf.update(z)
            self.fused_pos = self.ekf.get_state()[:2]
            self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw)
            self._update_navigation_path()
            self._send_position_udp()

    def _on_speed_update(self, speed):
        self.current_speed = speed
        self.ekf.predict(self.current_yaw, self.current_speed)
        self.fused_pos = self.ekf.get_state()[:2]
        self._update_navigation_path()
        self._send_position_udp()

    def _on_yaw_update(self, yaw):
        self.current_yaw = yaw
        self.map_viewer.move_to(*self.fused_pos, self.current_yaw)

    def _clear_rssi_cache(self):
        self.rssi_mutex.lock()
        self.rssi_data.clear()
        self.rssi_mutex.unlock()

    def _show_selection_dialog(self):
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room
            self.target_room = self.room_coords[selected]
            self._show_toast(f"<b>{selected}</b>로 안내를 시작합니다.")
            self.last_start_grid = None
            self._update_navigation_path()
        else:
            self._show_toast("안내를 취소했습니다.", duration=2000)
            self.target_room = None
            self.map_viewer.draw_path(None)
            
    def _start_ble_scan(self):
        if not self.ble_scanner_thread.isRunning():
            self.ble_scanner_thread.start()
            print("BLE Scan Started.")
    
    def load_stylesheet(self, filename):
        qss_file = QFile(filename)
        if qss_file.open(QFile.ReadOnly | QFile.Text):
            self.setStyleSheet(QTextStream(qss_file).readAll())
        else:
            print(f"'{filename}' 스타일시트 로드 실패!")

    def meters_to_grid(self, pos_m):
        row = int(pos_m[1] * self.config['px_per_m_y'] / self.BLOCK_SIZE)
        col = int(pos_m[0] * self.config['px_per_m_x'] / self.BLOCK_SIZE)
        return (row, col)

    def grid_to_pixels(self, pos_grid):
        px = pos_grid[1] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2
        py = pos_grid[0] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2
        return QPointF(px, py)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    config = load_config()
    main_window = IndoorPositioningApp(config)
    main_window.show()
    sys.exit(app.exec_())

