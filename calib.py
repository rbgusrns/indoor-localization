import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QGraphicsEllipseItem, QGraphicsPixmapItem
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPixmap

from app_config import load_config
from ble_scanner import BLEScanThread
from trilateration import KalmanFilter
from fingerprinting import FingerprintDB
from map_viewer import MapViewer

# 맵, 룸 크기, 그리드 설정
map_px_width = 800
map_px_height = 530
room_width_m = 10.8
room_height_m = 7.2
grid_width = 8
grid_height = 5

# 미터 → 픽셀 스케일 계산
px_per_m_x = map_px_width  / room_width_m    # ≈74px/m
px_per_m_y = map_px_height / room_height_m   # ≈73.6px/m

# 그리드 1칸 (m)
cell_m_x = room_width_m  / grid_width   # =1.35m
cell_m_y = room_height_m / grid_height  # =1.44m

class CalibViewer(MapViewer):
    def __init__(self, map_path, px_per_m_x, px_per_m_y):
        super().__init__(map_path, px_per_m_x, px_per_m_y)
        self.cfg = load_config()
        self.px_per_m_x = self.cfg['px_per_m_x']
        self.px_per_m_y = self.cfg['px_per_m_y']
        # 맵 이미지 로드
        pixmap = QPixmap(map_path)
        if pixmap.isNull():
            print(f"맵 파일 로드 실패: {map_path}")
        else:
            item = QGraphicsPixmapItem(pixmap)
            self.scene.addItem(item)
            self.scene.setSceneRect(QRectF(pixmap.rect()))
            self.setMinimumSize(int(pixmap.width()), int(pixmap.height()))
        self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        self.calib_marker = None

    def mark_calibration_point(self, x, y):
        if self.calib_marker:
            self.scene.removeItem(self.calib_marker)
        # 셀 크기(px) 계산
        cell_px_x = cell_m_x * self.px_per_m_x
        cell_px_y = cell_m_y * self.px_per_m_y
        cell_size_px = min(cell_px_x, cell_px_y)
        radius = cell_size_px * 0.1
        # 좌표 → 픽셀 변환
        px = x * cell_px_x + cell_px_x/2
        py = y * cell_px_y + cell_px_y/2
        marker = QGraphicsEllipseItem(px - radius, py - radius, radius*2, radius*2)
        marker.setBrush(Qt.blue)
        marker.setZValue(10)
        self.scene.addItem(marker)
        self.calib_marker = marker

class CalibrationWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.cfg = load_config()
        self.kf = {mac: KalmanFilter() for mac in self.cfg['beacon_macs']}
        self.fpdb = FingerprintDB(required_samples=20)
        self.thread = BLEScanThread(self.cfg, self.kf)
        self.thread.detected.connect(self.on_scan)

        # 그리드 및 상태 변수
        self.grid_width  = grid_width
        self.grid_height = grid_height
        self.current_x = 0
        self.current_y = 0
        # 임시 RSSI 벡터 저장소
        self.tmp_vec = {}

        map_path = self.cfg.get('map_file', '203.png')
        self.viewer = CalibViewer(map_path, px_per_m_x, px_per_m_y)
        self.viewer.mark_calibration_point(self.current_x, self.current_y)

        start_btn = QPushButton("Start Calibration")
        next_btn = QPushButton("Next Point")
        stop_btn = QPushButton("Stop and Save")

        start_btn.clicked.connect(self.thread.start)
        next_btn.clicked.connect(self.next_point)
        stop_btn.clicked.connect(self.finish)

        layout = QVBoxLayout(self)
        layout.addWidget(self.viewer)
        layout.addWidget(start_btn)
        layout.addWidget(next_btn)
        layout.addWidget(stop_btn)
        self.setLayout(layout)
        self.setWindowTitle("Fingerprint Calibration")
        self.showFullScreen()

    def next_point(self):
        self.current_x += 1
        if self.current_x >= self.grid_width:
            self.current_x = 0
            self.current_y += 1
            if self.current_y >= self.grid_height:
                self.current_y = 0
        # 다음 지점 준비: 임시 벡터 초기화
        self.tmp_vec.clear()
        self.viewer.mark_calibration_point(self.current_x, self.current_y)
        self.viewer.fitInView(self.viewer.scene.sceneRect(), Qt.KeepAspectRatio)

    def on_scan(self, vec):
        self.tmp_vec.update(vec)

        # 2개 이상 비콘만 수신되면 수집 시작
        if len(self.tmp_vec) >= 4:  # (여기 숫자 조절 가능)
            pos = (self.current_x, self.current_y)
            collected = self.fpdb.collect(pos, self.tmp_vec.copy())
            print(f"Collected @ {pos}: {self.tmp_vec}")
            self.tmp_vec.clear()

            if collected:
                print(f"저장 완료 @ {pos} (샘플 누적됨)")




    def finish(self):
        self.thread.stop()
        self.fpdb.build_index()
        path = self.cfg.get('fingerprint_db_path', 'fingerprint_db.json')
        self.fpdb.save(path)
        print("Calibration complete, DB saved to", path)
        QApplication.instance().quit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = CalibrationWindow()
    win.show()
    sys.exit(app.exec_())
