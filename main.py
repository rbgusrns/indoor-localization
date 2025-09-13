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

# --- 초기 설정 값 ---
INIT_X, INIT_Y = (0, 0)
INIT_YAW = 180.0

class IndoorPositioningApp(QWidget):
    def __init__(self, config):
        super().__init__()
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
        # --- 1. 토스트 알림 라벨 생성 ---
        # 레이아웃에 추가하지 않고, 부모 위젯(self)만 지정하여 화면 위에 떠 있도록 함
        self.toast_label = QLabel(self)
        self.toast_label.setObjectName("Toast")
        self.toast_label.setAlignment(Qt.AlignCenter)
        self.toast_label.hide() # 처음에는 숨김

        # 지도 뷰어 및 버튼 생성
        self.map_viewer = MapViewer(self.config['map_file'], self.config['px_per_m_x'], self.config['px_per_m_y'])
        self.map_viewer._init_est_items(INIT_X, INIT_Y, INIT_YAW)
        self.nav_btn = QPushButton("길안내")
        self.nav_btn.setObjectName("NAV")
        self.robot_btn = QPushButton("로봇\n호출")
        self.robot_btn.setObjectName("Robot")

        # 레이아웃 설정
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
        # ... (이전과 동일)
        self.ble_scanner_thread.detected.connect(self._on_ble_device_detected)
        if self.serial_reader:
            self.serial_reader.heading_received.connect(self._on_yaw_update)
            self.serial_reader.speed_received.connect(self._on_speed_update)
        self.nav_btn.clicked.connect(self._show_selection_dialog)
        shortcut = QShortcut(QKeySequence("G"), self)
        shortcut.activated.connect(self._start_ble_scan)

    def _start_timers(self):
        # ... (이전과 동일)
        self.rssi_clear_timer = QTimer(self)
        self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache)
        self.rssi_clear_timer.start(2000)

    # --- 토스트 알림 관련 메서드 ---
    def _show_toast(self, message, duration=3000):
        """화면 중앙 상단에 메시지를 일정 시간 동안 표시합니다."""
        self.toast_label.setText(message)
        self.toast_label.adjustSize() # 텍스트 크기에 맞게 라벨 크기 조절
        self._update_toast_position() # 중앙에 위치시키기
        self.toast_label.show()
        # duration 밀리초 후에 자동으로 hide 함수를 실행
        QTimer.singleShot(duration, self.toast_label.hide)

    def _update_toast_position(self):
        """토스트 알림을 창의 중앙 상단에 위치시킵니다."""
        # 창 가로 중앙, 상단에서 50px 아래에 위치
        x = (self.width() - self.toast_label.width()) / 2
        y = 50
        self.toast_label.move(int(x), int(y))

    def resizeEvent(self, event):
        """창 크기가 변경될 때마다 토스트 알림의 위치를 다시 계산합니다."""
        super().resizeEvent(event)
        self._update_toast_position()

    # --- 경로 및 위치 업데이트 로직 (이전과 동일) ---
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
    
    # ... (_on_ble_device_detected, _on_speed_update 등 나머지 함수는 이전과 동일)
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

    def _on_speed_update(self, speed):
        self.current_speed = speed
        self.ekf.predict(self.current_yaw, self.current_speed)
        self.fused_pos = self.ekf.get_state()[:2]
        self._update_navigation_path()

    def _on_yaw_update(self, yaw):
        self.current_yaw = yaw
        self.map_viewer.move_to(*self.fused_pos, self.current_yaw)

    def _clear_rssi_cache(self):
        self.rssi_mutex.lock()
        self.rssi_data.clear()
        self.rssi_mutex.unlock()

    def _show_selection_dialog(self):
        """'길안내' 버튼 클릭 시 토스트 알림을 통해 상태를 표시합니다."""
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room
            self.target_room = self.room_coords[selected]
            
            # --- 2. 토스트 알림 표시 ---
            self._show_toast(f"<b>{selected}</b>로 안내를 시작합니다.")
            
            self.last_start_grid = None
            self._update_navigation_path()
        else:
            # --- 3. 취소 시 토스트 알림 표시 및 경로 삭제 ---
            self._show_toast("안내를 취소했습니다.", duration=2000)
            self.target_room = None
            self.map_viewer.draw_path(None)
            
    def _start_ble_scan(self):
        if not self.ble_scanner_thread.isRunning():
            self.ble_scanner_thread.start()
            print("BLE Scan Started.")
    
    # ... (load_stylesheet, meters_to_grid, grid_to_pixels는 이전과 동일)
    def load_stylesheet(self, filename):
        qss_file = QFile(filename)
        if qss_file.open(QFile.ReadOnly | QFile.Text):
            self.setStyleSheet(QTextStream(qss_file).readAll())
            qss_file.close()

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

