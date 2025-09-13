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
    """
    실시간 실내 측위 및 동적 경로 안내 애플리케이션 클래스.
    """
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
        """핵심 로직(지도, 센서, DB 등) 객체들을 생성합니다."""
        self.binary_grid = create_binary_map(self.config['map_file'], block_size=self.BLOCK_SIZE)
        if self.binary_grid:
            print("이진 지도 생성 완료.")
            self.distance_map, self.max_dist = create_distance_map(self.binary_grid)
        else:
            print("오류: 이진 지도 생성에 실패했습니다.")
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
            print(f"시리얼 포트 오류: {e}. IMU 센서 없이 실행합니다.")
            self.serial_reader = None

    def _init_ui(self):
        """UI 위젯들을 생성하고 레이아웃을 설정합니다."""
        # --- ▼ 1. 상태 메시지 라벨 생성 ▼ ---
        self.nav_status_label = QLabel(self)
        self.nav_status_label.setObjectName("NavStatus") # QSS 스타일링을 위한 이름
        self.nav_status_label.setAlignment(Qt.AlignCenter)
        self.nav_status_label.hide() # 처음에는 숨김

        # 지도 뷰어 및 버튼 생성
        self.map_viewer = MapViewer(self.config['map_file'], self.config['px_per_m_x'], self.config['px_per_m_y'])
        self.map_viewer._init_est_items(INIT_X, INIT_Y, INIT_YAW)
        self.nav_btn = QPushButton("길안내")
        self.nav_btn.setObjectName("NAV")
        self.robot_btn = QPushButton("로봇\n호출")
        self.robot_btn.setObjectName("Robot")

        # 버튼 레이아웃
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.nav_btn)
        right_layout.addWidget(self.robot_btn)

        # 지도 + 버튼을 묶는 메인 콘텐츠 레이아웃
        content_layout = QHBoxLayout()
        content_layout.addWidget(self.map_viewer)
        content_layout.addLayout(right_layout)
        
        # --- ▼ 2. 최상위 레이아웃 구조 변경 ▼ ---
        # 상태 메시지 라벨을 맨 위에, 그 아래에 메인 콘텐츠를 배치
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(self.nav_status_label)
        main_layout.addLayout(content_layout)
        main_layout.setStretchFactor(content_layout, 1) # 지도 영역이 최대한 확장되도록 설정
        
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
        """'길안내' 버튼 클릭 시 목적지 설정 및 상태 메시지 표시/숨김 처리."""
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room
            self.target_room = self.room_coords[selected]
            print(f"'{selected}' 길안내 시작. 목표 좌표(m): {self.target_room}")
            
            # --- ▼ 3. 상태 메시지 업데이트 및 표시 ▼ ---
            self.nav_status_label.setText(f"현재 <font color='#3498db'>{selected}</font>로 안내중입니다.")
            self.nav_status_label.show()
            
            self.last_start_grid = None
            self._update_navigation_path()
        else:
            print("길안내 취소됨.")
            self.target_room = None
            self.map_viewer.draw_path(None)
            
            # --- ▼ 4. 상태 메시지 숨기기 ▼ ---
            self.nav_status_label.hide()
            
    def _start_ble_scan(self):
        if not self.ble_scanner_thread.isRunning():
            self.ble_scanner_thread.start()
            print("BLE Scan Started.")

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

