import joblib
import sys
import serial
import time
import numpy as np
import socket
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QShortcut, QLabel, QGraphicsDropShadowEffect
from PyQt5.QtCore import QTimer, QMutex, Qt, QFile, QTextStream, QPointF, QThread, pyqtSignal
from PyQt5.QtGui import QKeySequence, QColor

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
from robot_tracker import RobotTrackerThread
from lgbm_predictor import LGBM_Classifier_Predictor

# --- UDP 수신 스레드 클래스 ---
class UDPReceiverThread(QThread):
    message_received = pyqtSignal(str)
    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port, self.is_running = port, True
    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port)); sock.settimeout(1.0)
        print(f"UDP 수신 대기 시작 (포트: {self.port})")
        while self.is_running:
            try:
                data, _ = sock.recvfrom(1024)
                message = data.decode().strip()
                if message: self.message_received.emit(message)
            except socket.timeout: continue
        sock.close()
    def stop(self): self.is_running = False


# --- 메인 애플리케이션 클래스 ---
class IndoorPositioningApp(QWidget):
    def __init__(self, config):
        super().__init__()
        self.config, self.room_coords = config, {room['name']: (room['x'], room['y']) for room in config['rooms']}
        self.rssi_mutex, self.rssi_data = QMutex(), {}
        # fused_pos를 튜플이 아닌 NumPy 배열로 초기화
        self.current_speed, self.current_yaw, self.fused_pos = 0.0, 180.0, np.array([0.0, 0.0])
        self.target_room, self.last_start_grid, self.BLOCK_SIZE = None, None, 10

        # 벽 회피 기능 파라미터
        self.AVOIDANCE_THRESHOLD_GRID = 50
        self.CENTERING_STRENGTH = 5 # 그리드 중앙으로 보정하는 강도

        self.robot_arrival_processed = False

        self.udp_target_ip = self.config.get('udp_target_ip', "10.24.184.20")
        self.udp_target_port = self.config.get('udp_target_port', 5005)
        self.udp_socket, self.udp_send_timer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), QTimer(self)
        self.udp_destination_timer = QTimer(self)
        self.udp_receiver = UDPReceiverThread(port=self.config.get('udp_listen_port', 5006))

        # 벽 회피 보정을 위한 타이머
        self.wall_avoidance_timer = QTimer(self)


        self._init_logic_components(); self._init_ui(); self._connect_signals(); self._start_timers()

    def _init_logic_components(self):
        binary_grid_as_list = create_binary_map(self.config['map_file'], block_size=self.BLOCK_SIZE)
        if binary_grid_as_list is not None:
            self.binary_grid = np.array(binary_grid_as_list)
            dist_map_as_list, self.max_dist = create_distance_map(self.binary_grid)
            self.distance_map = np.array(dist_map_as_list)
        else:
            self.close()
        self.ekf = EKF(self.config.get('ekf_dt', 1.0))

        # LGBM Predictor 객체를 먼저 생성하고,
        # 그 객체의 load_model 메소드를 통해 모델과 전처리 정보를 모두 불러옵니다.
        self.lgbm_predictor = LGBM_Classifier_Predictor()
        if not self.lgbm_predictor.load_model('lgbm_predictor.pkl'):
            # load_model()이 파일을 못찾는 등 실패하면(False 반환), lgbm_predictor를 None으로 설정합니다.
            self.lgbm_predictor = None

        self.ble_scanner_thread = BLEScanThread(self.config)
        try:
            imu_port, baudrate = self.config.get('imu_port', '/dev/ttyUSB0'), self.config.get('imu_baudrate', 115200)
            self.serial_port = serial.Serial(imu_port, baudrate); time.sleep(1); self.serial_port.write(b'ZERO\n')
            self.serial_reader = SerialReader(port=imu_port, baudrate=baudrate); self.serial_reader.start()
        except serial.SerialException as e:
            print(f"시리얼 포트 오류: {e}."); self.serial_reader = None

        self.robot_tracker = RobotTrackerThread(port=self.config.get('robot_udp_port', 5005))

    def _init_ui(self):
        self.toast_label = QLabel(self); self.toast_label.setObjectName("Toast"); self.toast_label.setAlignment(Qt.AlignCenter); self.toast_label.hide()

        # 로봇 호출 상태 위젯
        self.setObjectName("MainWindow")
        self.robot_status_widget = QWidget(self)
        self.robot_status_widget.setObjectName("RobotStatus")
        self.robot_status_widget.hide()
        status_layout = QHBoxLayout(self.robot_status_widget)
        status_layout.setContentsMargins(25, 10, 25, 10); status_layout.setSpacing(20)
        status_layout.addWidget(QLabel("로봇이 오고 있습니다..."))
        self.stop_call_btn = QPushButton("중지"); self.stop_call_btn.setObjectName("StopCallButton")
        status_layout.addWidget(self.stop_call_btn)
        shadow = QGraphicsDropShadowEffect(); shadow.setBlurRadius(25); shadow.setColor(QColor(0, 0, 0, 80)); shadow.setOffset(0, 4)
        self.robot_status_widget.setGraphicsEffect(shadow)

        # 로봇 도착 확인 위젯
        self.arrival_prompt_widget = QWidget(self)
        self.arrival_prompt_widget.setObjectName("ArrivalPrompt")
        self.arrival_prompt_widget.hide()
        arrival_layout = QVBoxLayout(self.arrival_prompt_widget)
        arrival_layout.setContentsMargins(20, 20, 20, 20); arrival_layout.setSpacing(15); arrival_layout.setAlignment(Qt.AlignCenter)
        message_layout = QHBoxLayout(); message_layout.setSpacing(15); message_layout.setAlignment(Qt.AlignCenter)
        message_label = QLabel("로봇이 도착했습니다.<br>진료실로 이동하시겠습니까?")
        message_label.setAlignment(Qt.AlignCenter)
        message_layout.addWidget(message_label)
        self.confirm_move_btn = QPushButton("확인"); self.confirm_move_btn.setObjectName("ConfirmMoveButton")
        arrival_layout.addLayout(message_layout)
        arrival_layout.addWidget(self.confirm_move_btn, alignment=Qt.AlignCenter)
        arrival_shadow = QGraphicsDropShadowEffect(); arrival_shadow.setBlurRadius(25); arrival_shadow.setColor(QColor(0, 0, 0, 80)); arrival_shadow.setOffset(0, 4)
        self.arrival_prompt_widget.setGraphicsEffect(arrival_shadow)

        # 길안내 상태 위젯
        self.navigation_status_widget = QWidget(self)
        self.navigation_status_widget.setObjectName("NavigationStatus")
        self.navigation_status_widget.hide()
        nav_layout = QHBoxLayout(self.navigation_status_widget)
        nav_layout.setContentsMargins(25, 10, 25, 10); nav_layout.setSpacing(20)
        nav_layout.addWidget(QLabel("로봇을 따라 이동하세요.."))
        self.cancel_nav_btn = QPushButton("취소"); self.cancel_nav_btn.setObjectName("CancelNavButton")
        nav_layout.addWidget(self.cancel_nav_btn)
        nav_shadow = QGraphicsDropShadowEffect(); nav_shadow.setBlurRadius(25); nav_shadow.setColor(QColor(0, 0, 0, 80)); nav_shadow.setOffset(0, 4)
        self.navigation_status_widget.setGraphicsEffect(nav_shadow)

        # 현재 경로 안내 위젯 (좌측 상단)
        self.current_nav_widget = QWidget(self)
        self.current_nav_widget.setObjectName("CurrentNav")
        self.current_nav_widget.hide()
        nav_info_layout = QHBoxLayout(self.current_nav_widget)
        nav_info_layout.setContentsMargins(20, 10, 20, 10); nav_info_layout.setSpacing(15)
        self.current_nav_label = QLabel("안내: ")
        self.current_nav_cancel_btn = QPushButton("취소")
        self.current_nav_cancel_btn.setObjectName("CurrentNavCancelButton")
        nav_info_layout.addWidget(self.current_nav_label)
        nav_info_layout.addWidget(self.current_nav_cancel_btn)
        current_nav_shadow = QGraphicsDropShadowEffect(); current_nav_shadow.setBlurRadius(20); current_nav_shadow.setColor(QColor(0, 0, 0, 70)); current_nav_shadow.setOffset(0, 3)
        self.current_nav_widget.setGraphicsEffect(current_nav_shadow)

        # 메인 레이아웃
        self.map_viewer = MapViewer(self.config['map_file'], self.config['px_per_m_x'], self.config['px_per_m_y'])
        self.map_viewer._init_est_items(0, 0, 180.0)
        self.nav_btn = QPushButton("길안내"); self.nav_btn.setObjectName("NAV")
        self.robot_btn = QPushButton("로봇\n호출"); self.robot_btn.setObjectName("Robot")
        right_layout = QVBoxLayout(); right_layout.addWidget(self.nav_btn); right_layout.addWidget(self.robot_btn)
        main_layout = QHBoxLayout(self); main_layout.addWidget(self.map_viewer); main_layout.addLayout(right_layout)

        main_layout.setSpacing(13)

        self.setWindowTitle("ODIGA"); self.setFocusPolicy(Qt.StrongFocus); self.load_stylesheet('stylesheet.qss'); self.showFullScreen(); self.setFocus()

    def _connect_signals(self):
        self.ble_scanner_thread.detected.connect(self._on_ble_device_detected)

        if self.serial_reader:
            self.serial_reader.heading_received.connect(self._on_yaw_update)
            self.serial_reader.speed_received.connect(self._on_speed_update)
        self.nav_btn.clicked.connect(self._show_selection_dialog)
        self.robot_btn.clicked.connect(self._on_robot_call_clicked)
        self.stop_call_btn.clicked.connect(self._on_robot_call_stop_clicked)
        self.confirm_move_btn.clicked.connect(self._on_arrival_confirmed)
        self.cancel_nav_btn.clicked.connect(self._on_navigation_cancel_clicked)
        self.current_nav_cancel_btn.clicked.connect(lambda: self._stop_navigation("안내를 취소했습니다."))
        shortcut = QShortcut(QKeySequence("G"), self); shortcut.activated.connect(self._start_ble_scan)
        self.udp_send_timer.timeout.connect(self._send_position_udp)
        self.udp_destination_timer.timeout.connect(self._send_destination_udp)
        self.udp_receiver.message_received.connect(self._on_robot_message_received)
        self.robot_tracker.robot_position_updated.connect(self._on_robot_position_update)

        # 벽 회피 타이머의 timeout 신호를 _apply_wall_avoidance 메서드에 연결
        self.wall_avoidance_timer.timeout.connect(self._apply_wall_avoidance)

    # --- 키 입력 이벤트 핸들러 추가 ---
    def keyPressEvent(self, event):
        """키보드 입력을 처리합니다."""
        super().keyPressEvent(event)
        if event.key() == Qt.Key_R:
            print("'R' 키 입력 감지. pts_grid를 (2, 3)으로 수동 설정합니다.")
            try:
                pts_grid = (0, 0)
                print(f"🎯 모델 예측 그리드 (수동 설정): {pts_grid}")

                pts_pixels_qpoint = self.grid_to_pixels(pts_grid)

                px_per_m_x = self.config.get('px_per_m_x', 1.0)
                px_per_m_y = self.config.get('px_per_m_y', 1.0)
                # pts_meters 계산식 수정
                pts_meters = np.array([
                    pts_pixels_qpoint.x() * 19 / px_per_m_x,
                    pts_pixels_qpoint.y() * 19 / px_per_m_y
                ])
                print(f"수동 설정 pts_meters: {pts_meters}" )
                self.ekf.update(pts_meters)
                self.fused_pos = self.ekf.get_state()[:2].flatten()
                self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw)
                self._update_navigation_path()
            except Exception as e:
                print(f"수동 위치 설정 중 오류 발생: {e}")


    def _start_timers(self):
        self.rssi_clear_timer = QTimer(self); self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache); self.rssi_clear_timer.start(2000)
        self.udp_receiver.start()
        self.robot_tracker.start()


    def _on_robot_position_update(self, px, py):
        self.map_viewer.update_robot_position(px, py)

    def _send_position_udp(self):
        px, py = self.fused_pos[0] * self.config['px_per_m_x'], self.fused_pos[1] * self.config['px_per_m_y']
        message = f"{int(px)},{int(py)}"
        self.udp_socket.sendto(message.encode(), (self.udp_target_ip, self.udp_target_port))
        print(f"UDP Sent: {message}")

    def _send_destination_udp(self):
        """설정된 목적지(target_room)의 좌표를 UDP로 전송합니다."""
        if self.target_room:
            dest_m = self.target_room
            dest_px = dest_m[0] * self.config['px_per_m_x']
            dest_py = dest_m[1] * self.config['px_per_m_y']
            message = f"{int(dest_px)},{int(dest_py)}"
            self.udp_socket.sendto(message.encode(), (self.udp_target_ip, self.udp_target_port))
            print(f"UDP Destination Sent: {message}")
        else:
            self.udp_destination_timer.stop()
            print("오류: 목적지가 설정되지 않아 목적지 전송을 중단합니다.")

    def _show_toast(self, message, duration=3000):
        self.toast_label.setText(message); self.toast_label.adjustSize()
        self._update_popup_position(self.toast_label)
        self.toast_label.show(); self.toast_label.raise_()
        QTimer.singleShot(duration, self.toast_label.hide)

    def _update_popup_position(self, popup_widget):
        x = (self.width() - popup_widget.width()) / 2; y = 50
        popup_widget.move(int(x), int(y))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_popup_position(self.toast_label)
        self._update_popup_position(self.robot_status_widget)
        self._update_popup_position(self.arrival_prompt_widget)
        self._update_popup_position(self.navigation_status_widget)
        self.current_nav_widget.move(20, 20)

    def _update_navigation_path(self):
        if not self.target_room: return

        user_px = self.fused_pos[0] * self.config['px_per_m_x']
        user_py = self.fused_pos[1] * self.config['px_per_m_y']
        target_px = self.target_room[0] * self.config['px_per_m_x']
        target_py = self.target_room[1] * self.config['px_per_m_y']

        distance = np.sqrt((user_px - target_px)**2 + (user_py - target_py)**2)

        if distance < 25:
            self._stop_navigation("<b>목적지에 도착했습니다.</b> 안내를 종료합니다.")
            return

        start_m, end_m = self.fused_pos, self.target_room
        start_grid, end_grid = self.meters_to_grid(start_m), self.meters_to_grid(end_m)

        # 시작점이 맵 범위 안에 있는지 확인하는 안전 코드
        grid_height, grid_width = self.binary_grid.shape
        if not (0 <= start_grid[0] < grid_height and 0 <= start_grid[1] < grid_width):
            print(f"경고: 시작점 {start_grid}이(가) 맵 범위를 벗어났습니다. 경로를 업데이트하지 않습니다.")
            return # 함수를 즉시 종료하여 오류 방지

        if self.last_start_grid == start_grid: return
        self.last_start_grid = start_grid
        path_grid = find_path(self.binary_grid, start_grid, end_grid, self.distance_map, self.max_dist, self.config.get('penalty_strength', 2.5))
        path_pixels = [self.grid_to_pixels(pos) for pos in path_grid] if path_grid else None
        self.map_viewer.draw_path(path_pixels)

    def _get_direction_from_yaw(self, yaw):
        """Yaw 각도를 N, E, S, W 방향으로 변환합니다."""
        if 45 <= yaw < 135:
            return 'S'
        elif 135 <= yaw < 225:
            return 'W'
        elif 225 <= yaw < 315:
            return 'N'
        else: # 315 <= yaw or yaw < 45
            return 'E'

    def _on_ble_device_detected(self, rssi_vec):
        self.rssi_mutex.lock()
        self.rssi_data.update(rssi_vec)
        local_rssi_copy = self.rssi_data.copy()
        self.rssi_mutex.unlock()

        if len(local_rssi_copy) >= 6:
            if self.lgbm_predictor:
                try:
                    # 1. IMU 센서에서 받은 Yaw 값으로 현재 방향('N' 등)을 결정합니다.
                    direction = self._get_direction_from_yaw(self.current_yaw)
                    local_rssi_copy['direction'] = direction

                    # 2. Predictor 객체의 predict 메소드를 호출합니다. (내부에서 모든 전처리 수행)
                    predicted_label = self.lgbm_predictor.predict(local_rssi_copy)

                    # 3. 예측된 레이블('x_y')을 좌표로 변환합니다.
                    x_str, y_str = predicted_label.split('_')
                    pts_grid = (int(x_str)  , int(y_str) )
                    
                    print(f"🎯 모델 예측 그리드: {pts_grid}")
                    
                    pts_pixels_qpoint = self.grid_to_pixels(pts_grid)

                    px_per_m_x = self.config.get('px_per_m_x', 1.0)
                    px_per_m_y = self.config.get('px_per_m_y', 1.0)
                    # pts_meters 계산식 수정
                    pts_meters = np.array([
                        pts_pixels_qpoint.x() / px_per_m_x,
                        pts_pixels_qpoint.y() / px_per_m_y
                    ])
                    self.ekf.update(pts_meters)

                    self.fused_pos = self.ekf.get_state()[:2].flatten()

                    self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw)
                    self._update_navigation_path()

                except Exception as e:
                    print(f"LGBM 예측 중 오류 발생: {e}")


    def _on_speed_update(self, speed):
        self.current_speed = speed
        self.ekf.predict(self.current_yaw, self.current_speed)
        self.fused_pos = self.ekf.get_state()[:2].flatten()
        self._update_navigation_path()

    def _on_yaw_update(self, yaw):
        self.current_yaw = yaw
        self.map_viewer.move_to(*self.fused_pos, self.current_yaw)

    def _clear_rssi_cache(self):
        self.rssi_mutex.lock(); self.rssi_data.clear(); self.rssi_mutex.unlock()

    def _show_selection_dialog(self):
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room
            self.target_room = self.room_coords[selected]
            self._show_toast(f"<b>{selected}</b>로 안내를 시작합니다.")
            self.last_start_grid = None
            self._update_navigation_path()
            self.current_nav_label.setText(f"<b>{selected}</b> 안내중")
            self.current_nav_widget.adjustSize()
            self.current_nav_widget.show()
            self.current_nav_widget.raise_()
        else:
            self._show_toast("안내를 취소했습니다.", duration=2000)
            self.target_room = None
            self.map_viewer.draw_path(None)
            self.current_nav_widget.hide()

    def _start_ble_scan(self):
        if not self.ble_scanner_thread.isRunning():
            self.ble_scanner_thread.start()
            print("BLE Scan Started.")
            # BLE 스캔 시작과 함께 1초 간격으로 벽 회피 타이머를 시작
            if not self.wall_avoidance_timer.isActive():
                self.wall_avoidance_timer.start(1000) # 1000ms = 1초
                print("벽 회피 보정 타이머를 시작합니다 (1초 간격).")

    def _on_robot_call_clicked(self):
        if not self.target_room:
            self._show_toast("로봇을 부를 목적지를 선택해주세요.", duration=2500)

            dialog = SelectionDialog(self)
            if dialog.exec():
                selected = dialog.selected_room
                self.target_room = self.room_coords[selected]
                self.last_start_grid = None
                self._update_navigation_path()
                self.current_nav_label.setText(f"<b>{selected}</b> 안내중")
                self.current_nav_widget.adjustSize()
                self.current_nav_widget.show()
                self.current_nav_widget.raise_()
                self._show_toast(f"<b>{selected}</b>(으)로 목적지 설정 후 로봇을 호출합니다.")
            else:
                self._show_toast("로봇 호출을 취소했습니다.", duration=2000)
                return

        if self.udp_send_timer.isActive():
            self._show_toast("이미 로봇이 호출되었습니다.")
            return

        self.robot_arrival_processed = False

        self.udp_send_timer.start(1000)
        self.robot_status_widget.adjustSize()
        self._update_popup_position(self.robot_status_widget)
        self.robot_status_widget.show()
        self.robot_status_widget.raise_()

    def _on_robot_call_stop_clicked(self):
        if self.udp_send_timer.isActive():
            self.udp_send_timer.stop()
        if self.udp_destination_timer.isActive():
            self.udp_destination_timer.stop()
        self.robot_status_widget.hide()
        self.arrival_prompt_widget.hide()
        self.navigation_status_widget.hide()
        self._show_toast("로봇 호출을 중지했습니다.")

    def _on_arrival_confirmed(self):
        self.arrival_prompt_widget.hide()
        self.udp_destination_timer.start(500)
        self.navigation_status_widget.adjustSize()
        self._update_popup_position(self.navigation_status_widget)
        self.navigation_status_widget.show()
        self.navigation_status_widget.raise_()

    def _stop_navigation(self, message):
        """길안내를 중지하고 관련 UI를 정리합니다."""
        if self.udp_destination_timer.isActive():
            self.udp_destination_timer.stop()
        self.navigation_status_widget.hide()
        self.current_nav_widget.hide()
        self.target_room = None
        self.map_viewer.draw_path(None)
        self._show_toast(message)

    def _on_navigation_cancel_clicked(self):
        self._stop_navigation("길안내를 취소했습니다.")

    def _on_robot_message_received(self, message):
        print(f"로봇으로부터 메시지 수신: '{message}'")
        if message == "1":
            self.robot_status_widget.hide()
            self.arrival_prompt_widget.hide()
            self.navigation_status_widget.hide()
            if self.udp_send_timer.isActive():
                self.udp_send_timer.stop()
            self._show_toast("로봇이 도착했습니다.")

        elif message == "999,999":
            if not self.robot_arrival_processed:
                self.robot_arrival_processed = True
                print("로봇 도착 신호 (999,999) 수신. [최초 1회 처리]")

                if self.udp_send_timer.isActive():
                    self.udp_send_timer.stop()

                self.robot_status_widget.hide()

                self.arrival_prompt_widget.adjustSize()
                self._update_popup_position(self.arrival_prompt_widget)
                self.arrival_prompt_widget.show()
                self.arrival_prompt_widget.raise_()

    def load_stylesheet(self, filename):
        qss_file = QFile(filename);
        if qss_file.open(QFile.ReadOnly | QFile.Text): self.setStyleSheet(QTextStream(qss_file).readAll())
        else: print(f"'{filename}' 스타일시트 로드 실패!")

    def meters_to_grid(self, pos_m):
        row, col = int(pos_m[1] * self.config['px_per_m_y'] / self.BLOCK_SIZE), int(pos_m[0] * self.config['px_per_m_x'] / self.BLOCK_SIZE)
        return (row, col)

    def grid_to_pixels(self, pos_grid):
        px, py = pos_grid[1] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2, pos_grid[0] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2
        return QPointF(px, py)

    def _apply_wall_avoidance(self):
        """현재 위치가 벽에 너무 가까우면 현재 그리드의 중심으로 위치를 보정합니다."""
        if self.fused_pos is None or self.distance_map is None:
            return

        current_grid = self.meters_to_grid(self.fused_pos)
        row, col = current_grid

        height, width = self.distance_map.shape
        if not (0 <= row < height and 0 <= col < width):
            return

        distance_to_wall = self.distance_map[row][col]
        # 벽과의 거리가 설정된 임계값보다 가까울 때만 보정 로직을 실행합니다.
        if distance_to_wall >= self.AVOIDANCE_THRESHOLD_GRID:
            return

        # 1. 현재 그리드의 중심 픽셀 좌표를 계산합니다.
        center_pixels_qpoint = self.grid_to_pixels(current_grid)

        # 2. 중심 픽셀 좌표를 미터 단위로 변환합니다.
        center_m_x = center_pixels_qpoint.x() / self.config['px_per_m_x']
        center_m_y = center_pixels_qpoint.y() / self.config['px_per_m_y']
        center_pos_m = np.array([center_m_x, center_m_y])

        # 3. 현재 위치에서 그리드 중심으로 향하는 보정 벡터를 계산하고,
        #    보정 강도를 적용하여 이동량을 조절합니다.
        correction_vector_m = (center_pos_m - self.fused_pos) * self.CENTERING_STRENGTH

        # 4. fused_pos와 EKF 상태를 동시에 보정합니다.
        self.fused_pos += correction_vector_m
        try:
            self.ekf.x[0] = self.fused_pos[0]
            self.ekf.x[1] = self.fused_pos[1]
            
            if hasattr(self.ekf, "P"):
                # 위치 보정에 대한 불확실성을 약간 증가시켜 EKF가 새로운 측정에 더 잘 반응하도록 합니다.
                self.ekf.P[:2, :2] *= 1.2
                
        except Exception as e:
            print(f"EKF 상태 보정 중 오류 발생: {e}")

        # 5. 보정된 위치를 지도에 즉시 반영합니다.
        self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw)
        print(f"그리드 중앙 보정 적용: ({correction_vector_m[0]:.2f}, {correction_vector_m[1]:.2f})m 보정됨")

        # 6. 보정된 위치를 기반으로 경로를 다시 계산합니다.
        self._update_navigation_path()


    def closeEvent(self, event):
        self.robot_tracker.stop()
        self.udp_receiver.stop()
        self.ble_scanner_thread.stop()
        if self.serial_reader: self.serial_reader.stop()

        # 프로그램 종료 시 벽 회피 타이머 정지
        if self.wall_avoidance_timer.isActive():
            self.wall_avoidance_timer.stop()

        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    config = load_config()
    main_window = IndoorPositioningApp(config)
    sys.exit(app.exec_())