#멀티쓰레딩을 통해 RSSI 신호를 받는 파일. 

import sys
import asyncio
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from bleak import BleakScanner, BleakError
from collections import deque
from trilateration import SuperFilter
from app_config import load_config
from trilateration import KalmanFilter
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class BLEScanThread(QThread): # 멀티쓰레딩으로 RSSI 신호를 받고, 저장된 mac주소와 일치하다면 필터링 후 시그널 외부로 전달.
    detected = pyqtSignal(dict)
#   시그널 정의, 딕셔너리 인자로 저장
#   클래스 내부에서 발생한 신호를 외부로 전달!
#    main.py, calib.py에서 connect로 전달받음.

    def __init__(self, config, kalman_filters=None): #yaml 받고, 칼만필터 쓸 지 슈퍼필터 쓸 지 결정.
        super().__init__()
        self.config = config #yaml 파일 받기

        
        if kalman_filters is None: 
            self.filters = {mac: SuperFilter() for mac in config['beacon_macs']} #yaml 파일에 있는 mac 주소를 key로 하는 딕셔너리 생성 
        else:                                                                    #각 비콘마다 별도의 필터 객체 생성. filters : {mac: SuperFilter()...} key: mac이고 value는 SuperFilter 객체.
            self.filters = kalman_filters #객체를 받으면 그냥 씀.

        self.windows = {mac: deque(maxlen=config['filter_window']) for mac in config['beacon_macs']} #양방향 큐 선언. maxlwen = 5 각 비콘마다 크기가 5인 큐 선언..
        self.scanning = False

    def detection_callback(self, device, adv): #콜백함수. BLE 광고 패킷을 받을 때마다 호출된다..!!
        addr = device.address
        if addr in self.windows: #등록된 mac 주소와 같다면. 6개
            rssi = adv.rssi  
            
            #self.windows[addr].append(rssi)#주소 큐에 rssi값 추가. 
            #avg = sum(self.windows[addr]) / len(self.windows[addr]) #평균필터 -> 칼만필터 내부에서 처리해주므로 사용 X
            filt = self.filters[addr].filtering(rssi) #칼만필터 및 이동평균필터 적용. 
            # print(f"\"Detected {addr}: {rssi}, {filt}\",") 
            #self.detected.emit({addr: filt}) #주소와 필터링된 rssi값을 딕셔너리 형태로 emit. -> BLEScanThread 클래스의 detected 시그널을 발생시킴.
            self.detected.emit({addr: filt})
    async def scan_loop(self): #스캔 루프.
        scanner = BleakScanner() # 비동기 ble 라이브러리.
        scanner.register_detection_callback(self.detection_callback) #콜백함수 직접 등록. ->detection_callback
        try:
            await scanner.start() #start할때까지 await을 통해 대기.
            self.scanning = True #scan ON.
            while self.scanning: 
                await asyncio.sleep(self.config['scan_interval']) #주기 조정
        except BleakError as e:
            print(f"BLE scan error: {e}")
        finally:
            await scanner.stop()

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.scan_loop())
        finally:
            loop.close()

    def stop(self):
        self.scanning = False




class RSSIPlotter(QWidget):
    def __init__(self, config):
        super().__init__()
        self.setWindowTitle('Real-time RSSI Plot')
        self.resize(800, 600)

        # 데이터 저장용
        self.hist = {
            mac: deque(maxlen=100)
            for mac in config['beacon_macs']
        }

        # Matplotlib 캔버스
        self.canvas = FigureCanvas(Figure())
        self.ax = self.canvas.figure.subplots()
        self.ax.set_xlabel('sample index')
        self.ax.set_ylabel('RSSI (dBm)')
        self.ax.set_title('Beacon RSSI Plotter')

        # 레이아웃
        lay = QVBoxLayout()
        lay.addWidget(self.canvas)
        self.setLayout(lay)

        # 스캔 스레드
        self.thread = BLEScanThread(config)
        self.thread.detected.connect(self.on_rssi)  # 시그널 연결
        self.thread.start()

        # 0.1초마다 화면 갱신
        self.timer = self.canvas.figure.canvas.new_timer(interval=100, callbacks=[(self.redraw, (), {})])
        self.timer.start()

    def on_rssi(self, data: dict):
        # {'MAC': rssi} 형태로 옴
        for mac, rssi in data.items():
            self.hist[mac].append(rssi)

    def redraw(self):
        self.ax.clear()
        for mac, dq in self.hist.items():
            if dq:
                self.ax.plot(list(dq), label=mac)
        self.ax.legend(loc='upper right')
        self.ax.set_ylim(-80, -40)   # RSSI 범위 고정
        self.ax.set_xlabel('sample index')
        self.ax.set_ylabel('RSSI (dBm)')
        self.canvas.draw_idle()

    def closeEvent(self, e):
        self.thread.stop()
        self.thread.wait()
        super().closeEvent(e)





if __name__ == '__main__':
    config = load_config()
    app = QApplication(sys.argv) #pyqt 어플리케이션을 구현하기 위해 필수로 작성. 전반적인 설정 담당
    w = RSSIPlotter(config)
    w.show()
    sys.exit(app.exec_())