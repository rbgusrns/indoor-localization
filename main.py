import sys
import serial
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QShortcut
from PyQt5.QtCore import QTimer, QMutex, Qt, QFile, QTextStream, QPointF
from PyQt5.QtGui import QKeySequence

# --- 모듈 임포트 ---
# 필요한 모듈들이 실제 파일로 존재해야 합니다.
from app_config import load_config
from fingerprinting import FingerprintDB
from trilateration import EKF
from ble_scanner import BLEScanThread
from map_viewer import MapViewer
from serial_reader import SerialReader
from event import SelectionDialog
from bin import create_binary_map
# Astar 모듈에서 두 함수를 모두 임포트합니다.
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
        
        # --- 상태 변수 초기화 ---
        self.rssi_mutex = QMutex()
        self.rssi_data = {}
        self.current_speed = 0.0
        self.current_yaw = INIT_YAW
        self.fused_pos = (INIT_X, INIT_Y)
        self.target_room = None # 길안내 목표
        self.last_start_grid = None # 경로갱신 최적화용

        self.BLOCK_SIZE = 10

        # --- 초기화 메서드 순차 실행 ---
        self._init_logic_components()
        self._init_ui()
        self._connect_signals()
        self._start_timers()

    def _init_logic_components(self):
        """핵심 로직(지도, 센서, DB 등) 객체들을 생성합니다."""
        # 이진 지도 및 경로 탐색용 가중치 맵 생성
        self.binary_grid = create_binary_map(self.config['map_file'], block_size=self.BLOCK_SIZE)
        if self.binary_grid:
            print("이진 지도 생성 완료.")
            print("경로 탐색용 가중치 맵 생성 중...")
            self.distance_map, self.max_dist = create_distance_map(self.binary_grid)
            print("가중치 맵 생성 완료.")
        else:
            print("오류: 이진 지도 생성에 실패했습니다. 프로그램을 종료합니다.")
            self.close()

        self.ekf = EKF(self.config.get('ekf_dt', 1.0))
        self.fingerprint_db = FingerprintDB()
        self.fingerprint_db.load(self.config.get('fingerprint_db_path', 'fingerprint_db.json'))
        self.ble_scanner_thread = BLEScanThread(self.config)
        
        # 시리얼 포트 오류에 대비한 예외 처리
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
        """각종 컴포넌트의 시그널을 클래스의 메서드(슬롯)에 연결합니다."""
        self.ble_scanner_thread.detected.connect(self._on_ble_device_detected)
        
        if self.serial_reader:
            self.serial_reader.heading_received.connect(self._on_yaw_update)
            self.serial_reader.speed_received.connect(self._on_speed_update)
        
        self.nav_btn.clicked.connect(self._show_selection_dialog)
        shortcut = QShortcut(QKeySequence("G"), self)
        shortcut.activated.connect(self._start_ble_scan)

    def _start_timers(self):
        """주기적으로 실행될 타이머들을 설정하고 시작합니다."""
        self.rssi_clear_timer = QTimer(self)
        self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache)
        self.rssi_clear_timer.start(2000)

    # --- 경로 탐색 및 갱신 로직 ---
    def _update_navigation_path(self):
        """현재 위치를 기준으로 목표 지점까지의 경로를 다시 계산하고 지도에 표시합니다."""
        if not self.target_room:
            return

        start_m, end_m = self.fused_pos, self.target_room
        start_grid, end_grid = self.meters_to_grid(start_m), self.meters_to_grid(end_m)
        
        if self.last_start_grid == start_grid:
            return
        self.last_start_grid = start_grid

        print(f"경로 갱신: {start_grid} -> {end_grid}")
        penalty_strength = self.config.get('penalty_strength', 3.0)
        path_grid = find_path(
            self.binary_grid, start_grid, end_grid,
            self.distance_map, self.max_dist, penalty_strength
        )

        if path_grid:
            path_pixels = [self.grid_to_pixels(pos) for pos in path_grid]
            self.map_viewer.draw_path(path_pixels)
        else:
            self.map_viewer.draw_path(None) # 경로 탐색 실패 시 기존 경로 삭제

    # --- 슬롯 메서드 (이벤트 핸들러) ---
    def _on_ble_device_detected(self, rssi_vec):
        """BLE 신호가 감지되면 위치를 융합하고 경로를 갱신합니다."""
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
        """속도 정보가 수신되면 위치를 예측하고 경로를 갱신합니다."""
        self.current_speed = speed
        self.ekf.predict(self.current_yaw, self.current_speed)
        self.fused_pos = self.ekf.get_state()[:2]
        self._update_navigation_path()

    def _on_yaw_update(self, yaw):
        """방향 정보가 수신되면 UI의 사용자 아이콘을 회전시킵니다."""
        self.current_yaw = yaw
        self.map_viewer.move_to(*self.fused_pos, self.current_yaw)

    def _clear_rssi_cache(self):
        """오래된 RSSI 데이터를 주기적으로 삭제합니다."""
        self.rssi_mutex.lock()
        self.rssi_data.clear()
        self.rssi_mutex.unlock()

    def _show_selection_dialog(self):
        """'길안내' 버튼 클릭 시 목적지 설정 및 첫 경로 탐색을 시작합니다."""
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room
            self.target_room = self.room_coords[selected]
            print(f"'{selected}' 길안내 시작. 목표 좌표(m): {self.target_room}")
            self.last_start_grid = None # 경로 탐색 강제 실행을 위해 초기화
            self._update_navigation_path()
        else:
            print("선택이 취소되었습니다.")
            # 길안내 취소 시 목표 지점과 경로 삭제
            self.target_room = None
            self.map_viewer.draw_path(None)
            
    def _start_ble_scan(self):
        """'G' 키를 눌렀을 때 BLE 스캔을 시작합니다."""
        if not self.ble_scanner_thread.isRunning():
            self.ble_scanner_thread.start()
            print("BLE Scan Started.")

    # --- 유틸리티 메서드 ---
    def load_stylesheet(self, filename):
        """외부 QSS 파일을 읽어 스타일시트를 적용합니다."""
        qss_file = QFile(filename)
        if qss_file.open(QFile.ReadOnly | QFile.Text):
            self.setStyleSheet(QTextStream(qss_file).readAll())
            qss_file.close()

    def meters_to_grid(self, pos_m):
        """미터 단위 좌표를 그리드 좌표(행, 열)로 변환합니다."""
        row = int(pos_m[1] * self.config['px_per_m_y'] / self.BLOCK_SIZE)
        col = int(pos_m[0] * self.config['px_per_m_x'] / self.BLOCK_SIZE)
        return (row, col)

    def grid_to_pixels(self, pos_grid):
        """그리드 좌표(행, 열)를 픽셀 좌표(QPointF)로 변환합니다."""
        px = pos_grid[1] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2
        py = pos_grid[0] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2
        return QPointF(px, py)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    config = load_config()
    main_window = IndoorPositioningApp(config)
    main_window.show()
    sys.exit(app.exec_())

