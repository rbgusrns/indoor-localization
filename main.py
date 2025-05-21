import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QShortcut
from PyQt5.QtCore import QTimer, QMutex, Qt
from PyQt5.QtGui     import QKeySequence
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

    #BLE fingerprinting으로 위치 추정
    if len(on_detect.rssi) >= 6: #BLE 스캔으로 받은 RSSI가 일정 개수 이상일 때만
        
        pts, candidates , dists = fp.get_position(merged, k=k)
        pts = np.array(pts)
        #print(f"RSSI: {on_detect.rssi} BLE: {pts} ")
        print(f"BLE: {candidates} ")
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
        mv.mark_estimated_position(*fused_pos, yaw)
        
def speed_callback(speed = 0.0):
    global fused_pos
    on_detect.speed = speed
    
    ekf.predict(on_detect.yaw, speed)
    fused_pos = ekf.get_state()[:2]
    #mv.mark_estimated_position(*fused_pos,on_detect.yaw) #*fused_pos: 언패킹. 튜플이나 리스트이면 값을 풀어서 전달함.

def yaw_callback(yaw):
    global fused_pos
    on_detect.yaw = yaw
    on_detect.speed = 0.0
    #ekf.predict(on_detect.yaw, on_detect.speed)
    #fused_pos = ekf.get_state()[:2]
    #mv.mark_estimated_position(*temp_pos,on_detect.yaw)
    mv.move_to(*fused_pos,on_detect.yaw)
    
    

# 초기 속성
# INIT_X,INIT_Y = (1306,978) 픽셀
#INIT_X,INIT_Y = (37.5,37.5) -> 681,668
#INIT_X,INIT_Y = (71.93,54.91) -> (909,668)
INIT_X,INIT_Y = (53,40)
INIT_YAW = 90.0
# 초기 위치 설정

fused_pos = (INIT_X, INIT_Y) # 초기 위치 설정
temp_pos = (INIT_X, INIT_Y) #초기 위치 설정

on_detect.rssi = {}
on_detect.speed = 0.0
on_detect.yaw = INIT_YAW

if __name__ == "__main__":
    # 설정 로드
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    time.sleep(1)

    ser.write(b'ZERO\n') #시작시 0으로 초기화
    cfg = load_config()
    dt = cfg.get('ekf_dt', 1.0)
    
    # IMU 시리얼 리더 설정
    serial_reader = SerialReader(port=cfg.get('imu_port', '/dev/ttyUSB0'), baudrate=115200)


    #시리얼 콜백 설정 및 시작 !
    serial_reader.heading_received.connect(yaw_callback)
    serial_reader.speed_received.connect(speed_callback)
    serial_reader.start()

    # EKF 초기화
    ekf = EKF(dt)

    # Fingerprinting DB 로드
    fp = FingerprintDB()
    fp.load(cfg.get('fingerprint_db_path', 'fingerprint_db.json'))

    # PyQt 앱 구성
    app = QApplication(sys.argv) #어플리케이션 엔진 생성 sys.argv는 실행 옵션이며, 터미널에서 넘긴 인자를 저장
    widget = QWidget() # 빈 창을 생성.
    mv = MapViewer(cfg['map_file'], cfg['px_per_m_x'], cfg['px_per_m_y']) #맵 뷰어 객체 생성
    mv._init_est_items(INIT_X, INIT_Y,INIT_YAW) # 초기 위치
    #mv.update_heading(180.0)

    # BLE 스캐너 설정
    thread = BLEScanThread(cfg) #BLE 스캐너 쓰레드 객체 생성
    thread.detected.connect(lambda vec: on_detect( #connect: emit 받아서 실행 !
        vec, mv, fp, ekf, cfg['k_neighbors'],
        on_detect.speed, on_detect.yaw
    ))
    '''
    # UI 버튼 설정
    start_btn = QPushButton("Start Scan")
    stop_btn  = QPushButton("Stop Scan")
    start_btn.clicked.connect(thread.start) # scan_loop 시작! .start: 내부적으로 run 호출
    stop_btn.clicked.connect(thread.stop)

    # 레이아웃 구성
    layout = QVBoxLayout(widget) #위젯 위에 수직 레이아웃(구역) 생성.
    
    layout.addWidget(start_btn)
    layout.addWidget(stop_btn)
    widget.setLayout(layout) #위젯에 레이아웃 적용.
    '''
    layout = QVBoxLayout(widget)
    
    layout.addWidget(mv)
    widget.setLayout(layout) #위젯에 레이아웃 적용.
    widget.setFocusPolicy(Qt.StrongFocus) #위젯이 포커스를 받을 자격을 준다. 포커스 = 마우스로 클릭했을 때 그 상황. 
    widget.show() #버튼 두개와 맵을 담은 위젯을 보여줌.
    widget.showFullScreen()
    widget.setFocus() #위젯에게 포커스 부여.
    widget.setWindowTitle("ODIGA") #제목 설정
    
    shortcut = QShortcut(QKeySequence("G"), widget) # G키 단축키 지정
    shortcut.activated.connect(lambda: thread.start() if not thread.isRunning() else None) #G누를시 thread.start()로 연결

    # RSSI 초기화 타이머
    clear_timer = QTimer()
    def clear_rssi(): #오래된 RSSI 지우기
        rssi_mutex.lock()
        on_detect.rssi.clear()
        rssi_mutex.unlock()
    clear_timer.timeout.connect(clear_rssi)
    clear_timer.start(2000)
    
    '''
    update_timer = QTimer()
    def update_position():
        global fused_pos
        global temp_pos
        temp_pos = fused_pos # 속도 패킷이 들어왔을 때만 위치를 업데이트 하기위한 임시 position
        #print(f"EKF: {ekf.get_state()[:2]}")
        #mv.mark_estimated_position(*fused_pos, on_detect.yaw)
    
    update_timer.timeout.connect(update_position)
    update_timer.start(500) # 0.5초마다 위치 업데이트
    '''

    # 이벤트 루프 시작
    sys.exit(app.exec_())
    #app이 프로그램 엔진이므로, 이를 시작한다는 뜻.
    #app.exec_() : Qt의 이벤트 루프 시작. 시작하고 일하다가  프로그램 종료시 종료 코드를 반환하는데, (0이면 정상) 그 코드를 sys.exit()에 넘겨줌.
    #sys.exit() : 프로그램 종료. 종료 코드 반환.
