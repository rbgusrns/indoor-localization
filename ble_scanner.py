import asyncio
from PyQt5.QtCore import QThread, pyqtSignal
from bleak import BleakScanner, BleakError
from collections import deque
from trilateration import SuperFilter

class BLEScanThread(QThread): # ble 멀티쓰레딩
    detected = pyqtSignal(dict)
#   시그널 정의, 딕셔너리 인자로 저장
#   클래스 내부에서 발생한 신호를 외부로 전달!

    def __init__(self, config, kalman_filters=None): #yaml 받고, 칼만필터 쓸 지 슈퍼필터 쓸 지 결정.
        super().__init__()
        self.config = config #yaml 파일 받기

        
        if kalman_filters is None:
            self.filters = {mac: SuperFilter() for mac in config['beacon_macs']} #yaml 파일에 있는 mac 주소를 key로 하는 딕셔너리 생성 
        else:                                                                    #각 비콘마다 별도의 필터 객체 생성. filters : {mac: SuperFilter()...} key: mac이고 value는 SuperFilter 객체.
            self.filters = kalman_filters 

        self.windows = {mac: deque(maxlen=config['filter_window']) for mac in config['beacon_macs']} #양방향 큐 선언. maxlwen = 5 각 비콘마다 크기가 5인 큐 선언..
        self.scanning = False

    def detection_callback(self, device, adv): #콜백함수. BLE 광고 패킷을 받을 때마다 호출된다..!!
        addr = device.address
        if addr in self.windows: #등록된 mac 주소와 같다면.
            rssi = adv.rssi  
            #self.windows[addr].append(rssi)#주소 큐에 rssi값 추가. 
            #avg = sum(self.windows[addr]) / len(self.windows[addr]) #평균필터
            filt = self.filters[addr].filtering(rssi) #칼만필터 및 이동평균필터 적용. 
            self.detected.emit({addr: filt}) #주소와 필터링된 rssi값을 딕셔너리 형태로 emit. -> BLEScanThread 클래스의 detected 시그널을 발생시킴.

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
