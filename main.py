import sys
import serial
import time
import numpy as np
import socket
# ▼ math 라이브러리 추가
import math
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
        self.current_speed, self.current_yaw, self.fused_pos = 0.0, 180.0, (0,0)
        self.target_room, self.last_start_grid, self.BLOCK_SIZE = None, None, 10
        self.udp_target_ip = self.config.get('udp_target_ip', "10.24.184.20")
        self.udp_target_port = self.config.get('udp_target_port', 5005)
        self.udp_socket, self.udp_send_timer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), QTimer(self)
        self.udp_receiver = UDPReceiverThread(port=self.config.get('udp_listen_port', 5005))
        
        # ▼ 로봇 상태 저장을 위한 변수 추가
        self.robot_pixel_pos = None # 로봇의 현재 픽셀 좌표
        self.is_robot_arrived = False # 로봇 도착 여부 플래그
        
        self._init_logic_components(); self._init_ui(); self._connect_signals(); self._start_timers()

    def _init_logic_components(self):
        self.binary_grid = create_binary_map(self.config['map_file'], block_size=self.BLOCK_SIZE)
        if self.binary_grid: self.distance_map, self.max_dist = create_distance_map(self.binary_grid)
        else: self.close()
        self.ekf = EKF(self.config.get('ekf_dt', 1.0))
        self.fingerprint_db = FingerprintDB(); self.fingerprint_db.load(self.config.get('fingerprint_db_path', 'fingerprint_db.json'))
        self.ble_scanner_thread = BLEScanThread(self.config)
        try:
            imu_port, baudrate = self.config.get('imu_port', '/dev/ttyUSB0'), self.config.get('imu_baudrate', 115200)
            self.serial_port = serial.Serial(imu_port, baudrate); time.sleep(1); self.serial_port.write(b'ZERO\n')
            self.serial_reader = SerialReader(port=imu_port, baudrate=baudrate); self.serial_reader.start()
        except serial.SerialException as e: print(f"시리얼 포트 오류: {e}."); self.serial_reader = None
        
        self.robot_tracker = RobotTrackerThread(port=self.config.get('robot_udp_port', 5006))

    def _init_ui(self):
        self.toast_label = QLabel(self); self.toast_label.setObjectName("Toast"); self.toast_label.setAlignment(Qt.AlignCenter); self.toast_label.hide()
        
        self.robot_status_widget = QWidget(self)
        self.robot_status_widget.setObjectName("RobotStatus")
        self.robot_status_widget.hide()

        status_layout = QHBoxLayout(self.robot_status_widget)
        status_layout.setContentsMargins(25, 10, 25, 10)
        status_layout.setSpacing(20)

        # ▼ 위젯 내부 요소를 멤버 변수로 만들어 다른 함수에서 접근 가능하게 변경
        self.robot_status_label = QLabel("로봇이 오고 있습니다...")
        self.robot_action_btn = QPushButton("중지")
        self.robot_action_btn.setObjectName("StopCallButton")

        status_layout.addWidget(self.robot_status_label)
        status_layout.addWidget(self.robot_action_btn)

        shadow = QGraphicsDropShadowEffect(); shadow.setBlurRadius(25); shadow.setColor(QColor(0, 0, 0, 80)); shadow.setOffset(0, 4)
        self.robot_status_widget.setGraphicsEffect(shadow)

        self.map_viewer = MapViewer(self.config['map_file'], self.config['px_per_m_x'], self.config['px_per_m_y'])
        self.map_viewer._init_est_items(0, 0, 180.0)
        self.nav_btn = QPushButton("길안내"); self.nav_btn.setObjectName("NAV")
        self.robot_btn = QPushButton("로봇\n호출"); self.robot_btn.setObjectName("Robot")

        right_layout = QVBoxLayout(); right_layout.addWidget(self.nav_btn); right_layout.addWidget(self.robot_btn)
        main_layout = QHBoxLayout(self); main_layout.addWidget(self.map_viewer); main_layout.addLayout(right_layout)
        
        self.setWindowTitle("ODIGA"); self.setFocusPolicy(Qt.StrongFocus); self.load_stylesheet('stylesheet.qss'); self.showFullScreen(); self.setFocus()

    def _connect_signals(self):
        self.ble_scanner_thread.detected.connect(self._on_ble_device_detected)
        if self.serial_reader:
            self.serial_reader.heading_received.connect(self._on_yaw_update)
            self.serial_reader.speed_received.connect(self._on_speed_update)
        self.nav_btn.clicked.connect(self._show_selection_dialog)
        self.robot_btn.clicked.connect(self._on_robot_call_clicked)
        # ▼ 버튼 이름을 self.robot_action_btn 으로 변경
        self.robot_action_btn.clicked.connect(self._on_robot_call_stop_clicked)
        shortcut = QShortcut(QKeySequence("G"), self); shortcut.activated.connect(self._start_ble_scan)
        self.udp_send_timer.timeout.connect(self._send_position_udp)
        self.udp_receiver.message_received.connect(self._on_robot_message_received)
        self.robot_tracker.robot_position_updated.connect(self._on_robot_position_update)

    def _start_timers(self):
        self.rssi_clear_timer = QTimer(self); self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache); self.rssi_clear_timer.start(2000)
        self.udp_receiver.start()
        self.robot_tracker.start()

    # ▼ --- 신규 및 수정된 함수들 ---
    
    def _check_proximity(self):
        """사용자와 로봇의 거리를 확인하고, 가까우면 UI를 변경합니다."""
        if self.robot_pixel_pos is None or self.is_robot_arrived or not self.robot_status_widget.isVisible():
            return

        # 사용자 현재 위치를 픽셀로 변환
        user_px = self.fused_pos[0] * self.config['px_per_m_x']
        user_py = self.fused_pos[1] * self.config['px_per_m_y']
        
        # 로봇과 사용자 사이의 거리 계산
        distance = math.dist((user_px, user_py), self.robot_pixel_pos)

        # 거리가 50픽셀 이내이면 상태 변경
        if distance <= 50:
            self.is_robot_arrived = True
            print("로봇 도착! UI를 변경합니다.")
            self._transform_robot_status_widget()
    
    def _transform_robot_status_widget(self):
        """로봇 상태 위젯을 '도착' 상태로 변경합니다."""
        self.robot_status_label.setText("진료실로 이동하시겠습니까?")
        
        # 기존 '중지' 버튼의 연결을 끊고 '확인' 기능으로 변경
        self.robot_action_btn.setText("확인")
        try:
            self.robot_action_btn.clicked.disconnect(self._on_robot_call_stop_clicked)
        except TypeError:
            pass # 이미 연결이 끊겨있으면 무시
        self.robot_action_btn.clicked.connect(self._on_arrival_confirmed)
        
        # 위젯 배경색 변경 (스타일시트 직접 적용)
        self.robot_status_widget.setStyleSheet("""
            #RobotStatus {
                background-color: #27ae60; /* 초록색 계열 */
                border-radius: 20px;
            }
        """)
        self.robot_status_widget.adjustSize()
        self._update_popup_position(self.robot_status_widget)

    def _reset_robot_status_widget(self):
        """로봇 상태 위젯을 '호출 중' 기본 상태로 되돌립니다."""
        self.robot_status_label.setText("로봇이 오고 있습니다...")
        self.robot_action_btn.setText("중지")
        
        try:
            self.robot_action_btn.clicked.disconnect(self._on_arrival_confirmed)
        except TypeError:
            pass
        self.robot_action_btn.clicked.connect(self._on_robot_call_stop_clicked)
        
        # 위젯 배경색 원래대로 (qss 파일에 정의된 스타일을 따르도록 초기화)
        self.robot_status_widget.setStyleSheet("")
        self.is_robot_arrived = False
        self.robot_pixel_pos = None

    def _on_arrival_confirmed(self):
        """'확인' 버튼을 눌렀을 때의 동작"""
        self._show_toast("알겠습니다. 안내를 종료합니다.")
        self.robot_status_widget.hide()
        # 로봇 호출 관련 상태 초기화
        if self.udp_send_timer.isActive():
            self.udp_send_timer.stop()
        self._reset_robot_status_widget()

    def _on_robot_position_update(self, px, py):
        """로봇 트래커로부터 받은 픽셀 좌표로 지도 위 로봇 위치를 업데이트합니다."""
        self.robot_pixel_pos = (px, py) # 로봇 위치 저장
        self.map_viewer.update_robot_position(px, py)
        self._check_proximity() # 거리 확인

    # ▲ --- 신규 및 수정된 함수들 ---

    def _send_position_udp(self):
        px, py = self.fused_pos[0] * self.config['px_per_m_x'], self.fused_pos[1] * self.config['px_per_m_y']
        message = f"{int(px)},{int(py)}"
        self.udp_socket.sendto(message.encode(), (self.udp_target_ip, self.udp_target_port))
        print(f"UDP Sent: {message}")

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

    def _update_navigation_path(self):
        if not self.target_room: return
        start_m, end_m = self.fused_pos, self.target_room
        start_grid, end_grid = self.meters_to_grid(start_m), self.meters_to_grid(end_m)
        if self.last_start_grid == start_grid: return
        self.last_start_grid = start_grid
        path_grid = find_path(self.binary_grid, start_grid, end_grid, self.distance_map, self.max_dist, self.config.get('penalty_strength', 2.5))
        path_pixels = [self.grid_to_pixels(pos) for pos in path_grid] if path_grid else None
        self.map_viewer.draw_path(path_pixels)
    
    def _on_ble_device_detected(self, rssi_vec):
        self.rssi_mutex.lock(); self.rssi_data.update(rssi_vec); local_rssi_copy = self.rssi_data.copy(); self.rssi_mutex.unlock()
        if len(local_rssi_copy) >= 6:
            pts, _, _ = self.fingerprint_db.get_position(local_rssi_copy, k=self.config['k_neighbors'])
            self.ekf.update(np.array([pts[0], pts[1]])); self.fused_pos = self.ekf.get_state()[:2]
            self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw); self._update_navigation_path()
            self._check_proximity() # ▼ 사용자 위치 업데이트 시에도 거리 확인

    def _on_speed_update(self, speed):
        self.current_speed = speed; self.ekf.predict(self.current_yaw, self.current_speed); self.fused_pos = self.ekf.get_state()[:2]
        self._update_navigation_path()

    def _on_yaw_update(self, yaw):
        self.current_yaw = yaw; self.map_viewer.move_to(*self.fused_pos, self.current_yaw)

    def _clear_rssi_cache(self):
        self.rssi_mutex.lock(); self.rssi_data.clear(); self.rssi_mutex.unlock()

    def _show_selection_dialog(self):
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room; self.target_room = self.room_coords[selected]
            self._show_toast(f"<b>{selected}</b>로 안내를 시작합니다.")
            self.last_start_grid = None; self._update_navigation_path()
        else:
            self._show_toast("안내를 취소했습니다.", duration=2000)
            self.target_room = None; self.map_viewer.draw_path(None)
            
    def _start_ble_scan(self):
        if not self.ble_scanner_thread.isRunning(): self.ble_scanner_thread.start(); print("BLE Scan Started.")
    
    def _on_robot_call_clicked(self):
        if not self.udp_send_timer.isActive():
            self.udp_send_timer.start(1000)
            
            self._reset_robot_status_widget() # ▼ 위젯 상태 초기화
            self.robot_status_widget.adjustSize()
            self._update_popup_position(self.robot_status_widget)
            self.robot_status_widget.show(); self.robot_status_widget.raise_()
            self._show_toast("로봇을 호출했습니다.")
        else:
            self._show_toast("이미 로봇이 호출되었습니다.")

    def _on_robot_call_stop_clicked(self):
        if self.udp_send_timer.isActive():
            self.udp_send_timer.stop()
            self.robot_status_widget.hide()
            self._show_toast("로봇 호출을 중지했습니다.")
            self._reset_robot_status_widget() # ▼ 상태 초기화

    def _on_robot_message_received(self, message):
        print(f"로봇으로부터 메시지 수신: '{message}'")
        if message == "1":
            self.robot_status_widget.hide()
            self.udp_send_timer.stop()
            self._show_toast("로봇이 도착했습니다.")

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

    def closeEvent(self, event):
        self.robot_tracker.stop()
        self.udp_receiver.stop()
        self.ble_scanner_thread.stop()
        if self.serial_reader: self.serial_reader.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    config = load_config()
    main_window = IndoorPositioningApp(config)
    sys.exit(app.exec_())