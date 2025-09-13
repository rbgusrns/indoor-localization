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
        self.udp_target_ip = self.config.get('udp_target_ip', "192.168.0.141")
        self.udp_target_port = self.config.get('udp_target_port', 5005)
        self.udp_socket, self.udp_send_timer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), QTimer(self)
        self.udp_receiver = UDPReceiverThread(port=self.config.get('udp_listen_port', 5006))
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

    def _init_ui(self):
        self.toast_label = QLabel(self); self.toast_label.setObjectName("Toast"); self.toast_label.setAlignment(Qt.AlignCenter); self.toast_label.hide()
        
        self.robot_status_widget = QWidget(self)
        self.robot_status_widget.setObjectName("RobotStatus")
        self.robot_status_widget.hide()

        status_layout = QHBoxLayout(self.robot_status_widget)
        status_layout.setContentsMargins(25, 10, 25, 10)
        status_layout.setSpacing(20)

        status_text = QLabel("로봇이 오고 있습니다...")
        self.stop_call_btn = QPushButton("호출 중지")
        self.stop_call_btn.setObjectName("StopCallButton")

        status_layout.addWidget(status_text)
        status_layout.addWidget(self.stop_call_btn)

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
        self.stop_call_btn.clicked.connect(self._on_robot_call_stop_clicked)
        shortcut = QShortcut(QKeySequence("G"), self); shortcut.activated.connect(self._start_ble_scan)
        self.udp_send_timer.timeout.connect(self._send_position_udp)
        self.udp_receiver.message_received.connect(self._on_robot_message_received)

    def _start_timers(self):
        self.rssi_clear_timer = QTimer(self); self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache); self.rssi_clear_timer.start(2000)
        self.udp_receiver.start()

    def _send_position_udp(self):
        px, py = self.fused_pos[0] * self.config['px_per_m_x'], self.fused_pos[1] * self.config['px_per_m_y']
        message = f"px:{int(px)},py:{int(py)}"
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
        path_grid = find_path(self.binary_grid, start_grid, end_grid, self.distance_map, self.max_dist, self.config.get('penalty_strength', 10.0))
        path_pixels = [self.grid_to_pixels(pos) for pos in path_grid] if path_grid else None
        self.map_viewer.draw_path(path_pixels)
    
    def _on_ble_device_detected(self, rssi_vec):
        self.rssi_mutex.lock(); self.rssi_data.update(rssi_vec); local_rssi_copy = self.rssi_data.copy(); self.rssi_mutex.unlock()
        if len(local_rssi_copy) >= 6:
            pts, _, _ = self.fingerprint_db.get_position(local_rssi_copy, k=self.config['k_neighbors'])
            self.ekf.update(np.array([pts[0], pts[1]])); self.fused_pos = self.ekf.get_state()[:2]
            self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw); self._update_navigation_path()

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
            
            # --- ▼ 여기가 핵심 수정 사항 ▼ ---
            self.robot_status_widget.adjustSize() # 내용물에 맞게 컨테이너 크기 자동 조절
            # --- ▲ ---

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
        self.udp_receiver.stop()
        self.ble_scanner_thread.stop()
        if self.serial_reader: self.serial_reader.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    config = load_config()
    main_window = IndoorPositioningApp(config)
    sys.exit(app.exec_())

