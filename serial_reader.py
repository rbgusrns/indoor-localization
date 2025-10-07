#시리얼 통신 받는 파일.

from PyQt5.QtCore import QThread, pyqtSignal
import serial

class SerialReader(QThread):
    heading_received = pyqtSignal(float)  # heading
    speed_received = pyqtSignal(float)  # speed
    #두개의 시그널을 정의. 매개변수의 값을 외부로 전달한다고 선언. 
    def __init__(self, port="COM8", baudrate=115200):
        super().__init__()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if line.startswith("Y"):
                    try:
                        heading = (int(line[1:].strip()) / 10 - 90) % 360
                        #print("Heading:", heading)
                        self.heading_received.emit(heading)
                    except ValueError:
                        pass

                elif line.startswith("S"):
                    try:
                        speed = int(line[1:].strip()) / 10
                        #print("Speed:", speed)
                        self.speed_received.emit(speed)
                    except ValueError:
                        pass
            except Exception as e:
                print("UART Read error:", e)

    def stop(self):
        self.running = False
        self.ser.close()
