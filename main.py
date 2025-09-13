import sys
import serial
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QShortcut
from PyQt5.QtCore import QTimer, QMutex, Qt, QFile, QTextStream, QPointF
from PyQt5.QtGui import QKeySequence

# 아래 모듈들은 실제 파일이 존재해야 합니다.
from app_config import load_config
from fingerprinting import FingerprintDB
from trilateration import EKF
from ble_scanner import BLEScanThread
from map_viewer import MapViewer
from serial_reader import SerialReader
from event import SelectionDialog
from bin import create_binary_map
from Astar import find_path

# --- 초기 설정 값 ---
# INIT_X, INIT_Y = (36.5, 28)
INIT_X, INIT_Y = (0, 0)
INIT_YAW = 180.0

class IndoorPositioningApp(QWidget):
    """
    실내 측위 시스템의 메인 애플리케이션 클래스.
    UI, 데이터 처리, 상태 관리를 모두 캡슐화
    """
    def __init__(self, config):
        super().__init__()
        self.config = config

        self.room_coords = {room['name']: (room['x'], room['y']) for room in config['rooms']}
        
        # --- 상태 변수 및 동기화 객체 초기화 ---
        self.rssi_mutex = QMutex()
        self.rssi_data = {}
        self.current_speed = 0.0
        self.current_yaw = INIT_YAW
        self.fused_pos = (INIT_X, INIT_Y)

        self.BLOCK_SIZE = 10

        # --- 핵심 로직 컴포넌트 초기화 ---
        self._init_logic_components()
        
        # --- UI 초기화 ---
        self._init_ui()
        
        # --- 시그널-슬롯 연결 ---
        self._connect_signals()

        # --- 타이머 시작 ---
        self._start_timers()

    def _update_navigation_path(self):
        """현재 위치를 기준으로 목표 지점까지의 경로를 다시 계산하고 지도에 표시합니다."""
        # 1. 길안내 기능이 활성화 상태인지 확인 (목표 지점이 설정되었는지)
        if not hasattr(self, 'target_room') or self.target_room is None:
            return # 목표가 없으면 아무것도 안 함

        # 2. 현재 위치와 목표 위치를 그리드 좌표로 변환
        start_m = self.fused_pos
        end_m = self.target_room
        
        start_grid = self.meters_to_grid(start_m)
        end_grid = self.meters_to_grid(end_m)
        
        # (Optional) 현재 그리드 셀에서 변경이 없으면 계산을 건너뛰어 성능 향상
        if hasattr(self, 'last_start_grid') and self.last_start_grid == start_grid:
            return
        self.last_start_grid = start_grid # 현재 그리드 위치 저장

        print(f"경로 갱신: {start_grid} -> {end_grid}")

        # 3. A* 알고리즘으로 경로 탐색
        penalty_strength = self.config.get('penalty_strength', 10.0)
        path_grid = find_path(
            self.binary_grid, 
            start_grid, 
            end_grid,
            self.distance_map,
            self.max_dist,
            penalty_strength
        )

        # 4. 결과를 픽셀 좌표로 변환하여 지도에 그리기 (화살표 기능 포함)
        if path_grid and len(path_grid) >= 2:
            arrow_segment_grid = path_grid[-2:]
            main_path_grid = path_grid[:-1]
            main_path_pixels = [self.grid_to_pixels(pos) for pos in main_path_grid]
            arrow_segment_pixels = [self.grid_to_pixels(pos) for pos in arrow_segment_grid]
            self.map_viewer.draw_path_with_arrow(main_path_pixels, arrow_segment_pixels)
        elif path_grid:
            path_pixels = [self.grid_to_pixels(pos) for pos in path_grid]
            self.map_viewer.draw_path_with_arrow(path_pixels, None)
        else:
            self.map_viewer.draw_path_with_arrow(None, None)

    def _init_logic_components(self):
        """핵심 로직(EKF, Fingerprint DB, 스캐너 등) 객체들을 생성합니다."""

        self.binary_grid = create_binary_map(self.config['map_file'], block_size=self.BLOCK_SIZE)

        from Astar import create_distance_map 
        self.distance_map, self.max_dist = create_distance_map(self.binary_grid)

        if self.binary_grid:
            print("이진 지도 생성 완료.")
        else:
            print("오류: 이진 지도 생성에 실패했습니다.")

        self.ekf = EKF(self.config.get('ekf_dt', 1.0))
        
        self.fingerprint_db = FingerprintDB()
        self.fingerprint_db.load(self.config.get('fingerprint_db_path', 'fingerprint_db.json'))

        self.ble_scanner_thread = BLEScanThread(self.config)
        
        # IMU 시리얼 포트 설정 및 초기화
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        time.sleep(1)
        self.serial_port.write(b'ZERO\n')

        self.serial_reader = SerialReader(port=self.config.get('imu_port', '/dev/ttyUSB0'), baudrate=115200)
        self.serial_reader.start()

    def _init_ui(self):
        """UI 위젯들을 생성하고 레이아웃을 설정합니다."""
        self.map_viewer = MapViewer(self.config['map_file'], self.config['px_per_m_x'], self.config['px_per_m_y'])
        self.map_viewer._init_est_items(INIT_X, INIT_Y, INIT_YAW)
        
        # 버튼 생성
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
        
        # 윈도우 설정
        self.setWindowTitle("ODIGA")
        self.setFocusPolicy(Qt.StrongFocus)
        self.load_stylesheet('stylesheet.qss')
        self.showFullScreen()
        self.setFocus()

    def _connect_signals(self):
        """각종 컴포넌트의 시그널을 클래스의 메서드(슬롯)에 연결합니다."""
        # BLE 스캐너 시그널
        self.ble_scanner_thread.detected.connect(self._on_ble_device_detected)
        
        # 시리얼 리더 시그널
        self.serial_reader.heading_received.connect(self._on_yaw_update)
        self.serial_reader.speed_received.connect(self._on_speed_update)
        
        # UI 버튼 시그널
        self.nav_btn.clicked.connect(self._show_selection_dialog)

        # 단축키 설정
        shortcut = QShortcut(QKeySequence("G"), self)
        shortcut.activated.connect(self._start_ble_scan)

    def _start_timers(self):
        """주기적으로 실행될 타이머들을 설정하고 시작합니다."""
        self.rssi_clear_timer = QTimer(self)
        self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache)
        self.rssi_clear_timer.start(2000) # 2초마다 RSSI 캐시 초기화

    # --- 슬롯 메서드 (이벤트 핸들러) ---

    def _on_ble_device_detected(self, rssi_vec):
        """BLE 신호가 감지되었을 때 호출되는 슬롯."""
        self.rssi_mutex.lock()
        self.rssi_data.update(rssi_vec)
        local_rssi_copy = self.rssi_data.copy()
        self.rssi_mutex.unlock()

        if len(local_rssi_copy) >= 6:
            # 1. Fingerprinting으로 위치 추정
            pts, candidates, dists = self.fingerprint_db.get_position(local_rssi_copy, k=self.config['k_neighbors'])
            ble_pos = np.array(pts)
            print(f"BLE Candidates: {candidates}")

            # 2. EKF 업데이트
            z = np.array([ble_pos[0], ble_pos[1]])
            self.ekf.update(z)

            # 3. 융합 결과 획득 및 UI 업데이트
            self.fused_pos = self.ekf.get_state()[:2]
            self.map_viewer.update_debug(local_rssi_copy, self.fused_pos, np.array(dists))
            self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw) #현재 좌표와 방향을 맵에 출력함. *은 언패킹 연산자.

            self._update_navigation_path()

    def _on_speed_update(self, speed):
        """속도 정보가 수신되었을 때 호출되는 슬롯."""
        self.current_speed = speed
        self.ekf.predict(self.current_yaw, self.current_speed)
        self.fused_pos = self.ekf.get_state()[:2]
        # self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw)

        self._update_navigation_path()

    def _on_yaw_update(self, yaw):
        """방향(Yaw) 정보가 수신되었을 때 호출되는 슬롯."""
        self.current_yaw = yaw
        # EKF 예측은 속도 업데이트 시 수행되므로 여기서는 UI만 갱신
        self.map_viewer.move_to(*self.fused_pos, self.current_yaw)

    def _clear_rssi_cache(self):
        """오래된 RSSI 데이터를 주기적으로 삭제하는 슬롯."""
        self.rssi_mutex.lock()
        self.rssi_data.clear()
        self.rssi_mutex.unlock()
        # print("RSSI cache cleared.")

    def _show_selection_dialog(self):
        """'길안내' 버튼 클릭 시 진료실 선택 다이얼로그를 표시하는 슬롯."""
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room
            print(f"선택된 진료실: {selected}")
            
            # 1. 목표 지점 설정
            self.target_room = self.room_coords[selected]
            print(f"'{selected}' 길안내 시작. 목표 좌표(m): {self.target_room}")
            
            # 2. 첫 경로 계산 및 그리기
            self._update_navigation_path()

        else:
            print("선택이 취소되었습니다.")
            # 길안내 취소 시 목표 지점과 경로 삭제
            self.target_room = None
            self.map_viewer.draw_path_with_arrow(None, None)
            
    def _start_ble_scan(self):
        """'G' 키를 눌렀을 때 BLE 스캔을 시작합니다."""
        if not self.ble_scanner_thread.isRunning():
            self.ble_scanner_thread.start()
            print("BLE Scan Started.")

    def load_stylesheet(self, filename):
        """외부 QSS 파일을 읽어 위젯에 스타일시트를 적용합니다."""
        qss_file = QFile(filename)
        if qss_file.open(QFile.ReadOnly | QFile.Text):
            qss_stream = QTextStream(qss_file)
            self.setStyleSheet(qss_stream.readAll())
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
    
    # 설정 파일 로드
    config = load_config()
    
    # 메인 애플리케이션 인스턴스 생성 및 실행
    main_window = IndoorPositioningApp(config)
    main_window.show()
    
    sys.exit(app.exec_())