from PyQt5.QtCore import QThread, pyqtSignal
import serial

class SerialReader(QThread):
    received = pyqtSignal(float, float)  # speed heading

    def __init__(self, port="COM8", baudrate=115200):
        super().__init__()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if line.startswith("POS"):
                    _, speed, heading = line.split(',')
                    self.received.emit(float(speed), float(heading))
            except Exception as e:
                print("UART Read error:", e)

    def stop(self):
        self.running = False
        self.ser.close()
