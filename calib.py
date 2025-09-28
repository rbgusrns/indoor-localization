# 핑거프린팅 캘리브래이션 파일 (4방향 자동 측정 버전)

import sys
import os
from PyQt5.QtCore import QCoreApplication, Qt, QRectF, QTimer
from PyQt5.QtGui import QPixmap, QKeySequence, QColor, QFont
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QGraphicsEllipseItem, QGraphicsPixmapItem, QShortcut, QLabel # QLabel 임포트 추가

# --- 다른 모듈 import (실제 환경에 맞게 경로 설정 필요) ---
from app_config import load_config
from ble_scanner import BLEScanThread
from trilateration import KalmanFilter
from fingerprinting import FingerprintDB
from map_viewer import MapViewer

# --- 이 코드를 스크립트 최상단에 추가 (Qt 플러그인 오류 방지) ---
try:
    pyqt_dir = os.path.dirname(sys.modules['PyQt5'].__file__)
    plugin_path = os.path.join(pyqt_dir, "Qt5", "plugins")
    QCoreApplication.addLibraryPath(plugin_path)
except Exception as e:
    print(f"Qt 플러그인 경로 설정 중 오류 발생: {e}")
# ---------------------------------------------------------

# 맵, 룸 크기, 그리드 설정
map_px_width = 762
map_px_height = 574
room_width_m = 4
room_height_m = 3
grid_width = 4
grid_height = 3
BEACON_COUNT = 6

px_per_m_x = map_px_width / room_width_m
px_per_m_y = map_px_height / room_height_m

cell_m_x = room_width_m / grid_width
cell_m_y = room_height_m / grid_height


class CalibViewer(MapViewer):
    def __init__(self, map_path, px_per_m_x, px_per_m_y):
        super().__init__(map_path, px_per_m_x, px_per_m_y)
        self.cfg = load_config()
        self.px_per_m_x = self.cfg['px_per_m_x']
        self.px_per_m_y = self.cfg['px_per_m_y']
        
        pixmap = QPixmap(map_path)
        if pixmap.isNull():
            print(f"맵 파일 로드 실패: {map_path}")
            sys.exit(1)
        
        item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(item)
        self.scene.setSceneRect(QRectF(pixmap.rect()))
        self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

        self.calib_marker = None
        self.completed_markers = []

    def mark_calibration_point(self, x, y, direction, completed_directions):
        if self.calib_marker:
            self.scene.removeItem(self.calib_marker)
        for marker in self.completed_markers:
            self.scene.removeItem(marker)
        self.completed_markers.clear()

        cell_px_x = cell_m_x * self.px_per_m_x
        cell_px_y = cell_m_y * self.px_per_m_y
        
        for completed_dir in completed_directions:
            offset_x, offset_y = 0, 0
            radius_small = (min(cell_px_x, cell_px_y)) * 0.05
            if completed_dir == 'N': offset_y = -cell_px_y * 0.25
            elif completed_dir == 'S': offset_y = cell_px_y * 0.25
            elif completed_dir == 'W': offset_x = -cell_px_x * 0.25
            elif completed_dir == 'E': offset_x = cell_px_x * 0.25
            
            px_center = x * cell_px_x + cell_px_x / 2
            py_center = y * cell_px_y + cell_px_y / 2
            
            comp_marker = QGraphicsEllipseItem(px_center + offset_x - radius_small, py_center + offset_y - radius_small, radius_small * 2, radius_small * 2)
            comp_marker.setBrush(Qt.gray)
            comp_marker.setZValue(11)
            self.scene.addItem(comp_marker)
            self.completed_markers.append(comp_marker)

        colors = {'N': Qt.blue, 'E': Qt.green, 'S': Qt.red, 'W': QColor('orange')}
        radius_large = (min(cell_px_x, cell_px_y)) * 0.1
        px = x * cell_px_x + cell_px_x/2
        py = y * cell_px_y + cell_px_y/2
        
        self.calib_marker = QGraphicsEllipseItem(px - radius_large, py - radius_large, radius_large * 2, radius_large * 2)
        self.calib_marker.setBrush(colors.get(direction, Qt.black))
        self.calib_marker.setZValue(10)
        self.scene.addItem(self.calib_marker)


class CalibrationWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.cfg = load_config()
        self.kf = {mac: KalmanFilter() for mac in self.cfg['beacon_macs']}
        self.fpdb = FingerprintDB(required_samples=50)
        self.thread = BLEScanThread(self.cfg, self.kf)
        self.thread.detected.connect(self.on_scan)

        self.grid_width = grid_width
        self.grid_height = grid_height
        self.current_x = 0
        self.current_y = 0
        
        self.directions = ['N', 'E', 'S', 'W']
        self.current_direction = self.directions[0]
        self.completed_directions = set()

        self.tmp_vec = {}

        map_path = self.cfg.get('map_file', 'map.png')
        self.viewer = CalibViewer(map_path, px_per_m_x, px_per_m_y)
        self.update_marker()
        self.setFocusPolicy(Qt.StrongFocus)

        self.setup_shortcuts()
        
        layout = QVBoxLayout(self)
        layout.addWidget(self.viewer)

        ### --- 수정/추가된 부분 --- ###
        # 상태 표시를 위한 QLabel 위젯 추가
        self.status_label = QLabel("준비 완료. 'G'를 눌러 스캔을 시작하세요.")
        font = self.status_label.font()
        font.setPointSize(12)
        self.status_label.setFont(font)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        ### -------------------------- ###

        self.scan_button = QPushButton("Start Scan (G)")
        self.scan_button.clicked.connect(self.start_scan)
        layout.addWidget(self.scan_button)

        self.setLayout(layout)
        self.setWindowTitle("Fingerprint Calibration (4-Direction Auto)")

    def setup_shortcuts(self):
        QShortcut(QKeySequence("G"), self).activated.connect(self.start_scan)
        QShortcut(QKeySequence("X"), self).activated.connect(self.stop_scan)
        QShortcut(QKeySequence("N"), self).activated.connect(self.move_to_next_point)
        QShortcut(QKeySequence("Q"), self).activated.connect(self.finish)
        
        QShortcut(QKeySequence(Qt.Key_Up), self).activated.connect(lambda: self.change_direction('N'))
        QShortcut(QKeySequence(Qt.Key_Down), self).activated.connect(lambda: self.change_direction('S'))
        QShortcut(QKeySequence(Qt.Key_Left), self).activated.connect(lambda: self.change_direction('W'))
        QShortcut(QKeySequence(Qt.Key_Right), self).activated.connect(lambda: self.change_direction('E'))

    def update_marker(self):
        self.viewer.mark_calibration_point(self.current_x, self.current_y, self.current_direction, self.completed_directions)

    def change_direction(self, direction):
        if not self.thread.isRunning() and direction in self.directions:
            self.current_direction = direction
            print(f"Direction changed to: {self.current_direction}")
            self.update_marker()

    def start_scan(self):
        if not self.thread.isRunning() and self.current_direction not in self.completed_directions:
            print(f"pos: ({self.current_x},{self.current_y}), dir: {self.current_direction} - SCAN START!!")
            
            ### --- 수정/추가된 부분 --- ###
            # 스캔 시작 시 상태 메시지 업데이트
            status_text = f"({self.current_x}, {self.current_y}, '{self.current_direction}') 수집 중: 0 / {self.fpdb.required_samples}"
            self.status_label.setText(status_text)
            ### -------------------------- ###

            self.thread.start()
            self.scan_button.setEnabled(False)

    def stop_scan(self):
        if self.thread.isRunning():
            self.thread.stop()
            self.scan_button.setEnabled(True)
            print("Scan stopped by user.")
            
            ### --- 수정/추가된 부분 --- ###
            self.status_label.setText("사용자에 의해 스캔이 중지되었습니다.")
            ### -------------------------- ###

    def move_to_next_point(self):
        if not self.thread.isRunning():
            self.current_x += 1
            if self.current_x >= self.grid_width:
                self.current_x = 0
                self.current_y += 1
                if self.current_y >= self.grid_height:
                    self.current_y = 0
            
            self.completed_directions.clear()
            self.current_direction = self.directions[0]
            self.tmp_vec.clear()
            self.scan_button.setEnabled(True)
            self.update_marker()
            print(f"--- Moved to next point: ({self.current_x}, {self.current_y}) ---")
            
            ### --- 수정/추가된 부분 --- ###
            status_text = f"({self.current_x}, {self.current_y})로 이동. 'G'를 눌러 스캔을 시작하세요."
            self.status_label.setText(status_text)
            ### -------------------------- ###

    def on_scan(self, vec):
        self.tmp_vec.update(vec)

        if len(self.tmp_vec) >= BEACON_COUNT:
            pos_key = (self.current_x, self.current_y, self.current_direction)
            
            ### --- 수정/추가된 부분 --- ###
            # collect를 호출하기 전에 현재 카운트를 가져와서 텍스트 업데이트
            # 버퍼에 아직 추가되기 전이므로 +1을 해준다.
            current_count = len(self.fpdb._acc_buffer.get(pos_key, [])) + 1
            total_samples = self.fpdb.required_samples
            status_text = f"({self.current_x}, {self.current_y}, '{self.current_direction}') 수집 중: {current_count} / {total_samples}"
            self.status_label.setText(status_text)
            ### -------------------------- ###
            
            collected = self.fpdb.collect(pos_key, self.tmp_vec.copy())
            self.tmp_vec.clear()

            if collected:
                print(f"수집 완료 @ {pos_key}")
                self.thread.stop()
                QTimer.singleShot(100, self._on_collection_finished)

    def _on_collection_finished(self):
        self.completed_directions.add(self.current_direction)
        remaining_dirs = [d for d in self.directions if d not in self.completed_directions]
        
        if remaining_dirs:
            print(f"'{self.current_direction}' 방향 완료. 다음 방향 '{remaining_dirs[0]}' 스캔을 시작합니다.")
            self.change_direction(remaining_dirs[0])
            self.start_scan() 
        else:
            print(f"({self.current_x}, {self.current_y})의 4방향 스캔 완료. 다음 지점으로 이동합니다.")
            
            ### --- 수정/추가된 부분 --- ###
            status_text = f"({self.current_x}, {self.current_y}) 모든 방향 수집 완료! 'N'으로 이동하세요."
            self.status_label.setText(status_text)
            self.scan_button.setEnabled(True) # 'N'을 누르기 전에 수동으로 재시작할 경우를 위해 버튼 활성화
            ### -------------------------- ###
            
            # self.move_to_next_point() # 자동 이동 대신 대기하도록 변경 (사용자 편의성)
            # 만약 모든 방향 완료 후 자동으로 다음 지점으로 이동하게 하려면 아래 주석을 푸세요.
            # self.move_to_next_point()

    def finish(self):
        if not self.thread.isRunning():
            self.thread.stop()
            self.fpdb.build_index()
            path = self.cfg.get('fingerprint_db_path', 'fingerprint_db_4dir.json')
            self.fpdb.save(path)
            print(f"Calibration complete, DB saved to {path}")
            QApplication.instance().quit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = CalibrationWindow()
    win.show()
    win.setFocus()
    sys.exit(app.exec_())