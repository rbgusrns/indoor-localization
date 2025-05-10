import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtCore import QTimer, QMutex
from app_config import load_config
from fingerprinting import FingerprintDB
from trilateration import EKF
from ble_scanner import BLEScanThread
from map_viewer import MapViewer
import numpy as np
from serial_reader import SerialReader
import serial
import time
# RSSI 보호용 뮤텍스
rssi_mutex = QMutex()
#mutex: 한번에 하나의 스레드만 이 자원에 접근할 수 있도록 함
#먼소리냐..

# on_detect 콜백
def on_detect(rssi_vec, mv, fp, ekf, k, speed, yaw):
    # 1) RSSI 누적 (스레드 안전)
    rssi_mutex.lock()
    merged = getattr(on_detect, 'rssi', {}).copy() 
    #getattr(): on_detect.rssi 반환, 없다면 빈 딕셔너리 반환
    #merged에 기존 RSSI 복사 .copy()
    merged.update(rssi_vec) # .update(): 새로운 RSSI 추가
    on_detect.rssi = merged # 기존 딕셔너리에 업데이트
    
    rssi_mutex.unlock()

    # 2) EKF 예측 단계
    #ekf.predict(yaw, speed)

    # 3) BLE fingerprinting으로 위치 추정
    if len(on_detect.rssi) >= 4: #BLE 스캔으로 받은 RSSI가 3개 이상일 때만
        
        pts, candidates , dists = fp.get_position(merged, k=k)
        pts = np.array(pts)
        print(f"RSSI: {on_detect.rssi} BLE: {pts} ")
        #print(f"BLE: {pts}")
        dists = np.array(dists)
        #weights = 1.0 / (dists + 1e-5)
        #ble_pos = np.average(pts, axis=0, weights=weights)
        ble_pos = pts
        #print(f"{ble_pos}")
        # 4) EKF 업데이트 단계
        z = np.array([ble_pos[0], ble_pos[1]])
        ekf.update(z)

        # 5) 융합 결과 얻기
        fused_pos = ekf.get_state()[:2]
        #print(f"{fused_pos}")
        # 6) UI 업데이트
        mv.update_debug(merged, fused_pos, dists)
        mv.mark_estimated_position(*fused_pos)

# 초기 속성
on_detect.rssi = {}
on_detect.speed = 0.0
on_detect.yaw = 0.0

if __name__ == "__main__":
    # 설정 로드
    #ser = serial.Serial('/dev/ttyUSB0', 115200)
    time.sleep(1)

    #ser.write(b'ZERO\n')
    cfg = load_config()
    dt = cfg.get('ekf_dt', 1.0)
    
    # IMU 시리얼 리더 설정
    serial_reader = SerialReader(port=cfg.get('imu_port', 'COM8'), baudrate=115200)

    # IMU 콜백: 속도, yaw(deg)만 받아옴
    def imu_callback(speed, yaw):
        on_detect.speed = speed
        on_detect.yaw = yaw
        
        ekf.predict(yaw, speed)
        fused_pos = ekf.get_state()[:2]
        mv.mark_estimated_position(*fused_pos, yaw)
    serial_reader.received.connect(imu_callback)
    serial_reader.start()

    # EKF 초기화
    ekf = EKF(dt)

    # Fingerprinting DB 로드
    fp = FingerprintDB()
    fp.load(cfg.get('fingerprint_db_path', 'fingerprint_db.json'))

    # PyQt 앱 구성
    app = QApplication(sys.argv)
    widget = QWidget()
    mv = MapViewer(cfg['map_file'], cfg['px_per_m_x'], cfg['px_per_m_y'])
    mv.mark_estimated_position(0, 0, 0.0)

    # BLE 스캐너 설정
    thread = BLEScanThread(cfg)
    thread.detected.connect(lambda vec: on_detect( #connect: emit 받아서 실행 !
        vec, mv, fp, ekf, cfg['k_neighbors'],
        on_detect.speed, on_detect.yaw
    ))

    # UI 버튼 설정
    start_btn = QPushButton("Start Scan")
    stop_btn  = QPushButton("Stop Scan")
    start_btn.clicked.connect(thread.start) # scan_loop 시작! .start: 내부적으로 run 호출
    stop_btn.clicked.connect(thread.stop)

    # 레이아웃 구성
    layout = QVBoxLayout(widget)
    layout.addWidget(mv)
    layout.addWidget(start_btn)
    layout.addWidget(stop_btn)
    widget.setLayout(layout)
    widget.show()

    # RSSI 초기화 타이머
    clear_timer = QTimer()
    def clear_rssi(): #오래된 RSSI 지우기
        rssi_mutex.lock()
        on_detect.rssi.clear()
        rssi_mutex.unlock()
    clear_timer.timeout.connect(clear_rssi)
    clear_timer.start(2000)

    # 이벤트 루프 시작
    sys.exit(app.exec_())
