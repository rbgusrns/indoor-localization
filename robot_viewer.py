import socket
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

class RobotTrackerThread(QThread):
    """
    UDP를 통해 로봇의 좌표를 수신하고, Numpy를 이용해 픽셀 좌표로 변환하여 시그널을 발생시키는 스레드.
    OpenCV를 사용하지 않습니다.
    """
    robot_position_updated = pyqtSignal(float, float)

    def __init__(self, port=5006, parent=None):
        super().__init__(parent)
        self.port = port
        self.is_running = True

        # 원본 좌표계 (로봇 실제 좌표)
        # 순서: 좌상단, 좌하단, 우상단, 우하단
        src_points = np.array([
            [1, -1.8],    # 왼쪽 위
            [3.7, -0.3],  # 왼쪽 아래
            [-1, 2],      # 오른쪽 위
            [1.5, 3.2]    # 오른쪽 아래
        ])

        # 대상 좌표계 (픽셀 좌표)
        # 순서: 좌상단, 좌하단, 우상단, 우하단
        dst_points = np.array([
            [0, 0],       # 왼쪽 위
            [0, 574],     # 왼쪽 아래
            [762, 0],     # 오른쪽 위
            [762, 574]    # 오른쪽 아래
        ])
        
        # 투영 변환 행렬 계산
        self.transform_matrix = self._calculate_perspective_transform(src_points, dst_points)

    def _calculate_perspective_transform(self, src, dst):
        """
        4개의 대응점을 이용하여 투영 변환 행렬(Homography)을 계산합니다.
        이는 Ax=B 형태의 선형방정식 시스템을 푸는 것과 같습니다.
        """
        A = []
        B = []
        for i in range(4):
            x, y = src[i]
            xp, yp = dst[i]
            A.append([x, y, 1, 0, 0, 0, -x*xp, -y*xp])
            A.append([0, 0, 0, x, y, 1, -x*yp, -y*yp])
            B.append(xp)
            B.append(yp)
        
        A = np.array(A)
        B = np.array(B)
        
        # np.linalg.solve를 사용하여 8개의 계수(h11~h32)를 구합니다.
        try:
            h = np.linalg.solve(A, B)
            # 3x3 변환 행렬로 재구성 (h33는 1로 고정)
            h_matrix = np.append(h, 1).reshape(3, 3)
            return h_matrix
        except np.linalg.LinAlgError:
            print("행렬 계산 오류: 특이 행렬(singular matrix)일 수 있습니다.")
            return np.identity(3) # 오류 발생 시 단위 행렬 반환

    def transform_coordinates(self, x, y):
        """계산된 변환 행렬을 이용해 좌표를 변환합니다."""
        # 동차좌표계(Homogeneous Coordinates) 벡터 생성
        src_vec = np.array([x, y, 1])
        
        # 행렬 곱셈으로 변환
        dst_vec = self.transform_matrix @ src_vec
        
        # 동차좌표를 2D 좌표로 변환 (z값으로 나누기)
        if dst_vec[2] != 0:
            return dst_vec[0] / dst_vec[2], dst_vec[1] / dst_vec[2]
        else:
            return 0, 0

    def run(self):
        """스레드 실행 시 호출되는 메인 루프"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port))
        sock.settimeout(1.0)
        print(f"로봇 위치 UDP 수신 대기 시작 (포트: {self.port})")

        while self.is_running:
            try:
                data, _ = sock.recvfrom(1024)
                message = data.decode().strip()
                
                if message:
                    parts = message.split(',')
                    if len(parts) == 2:
                        x, y = float(parts[0]), float(parts[1])
                        px, py = self.transform_coordinates(x, y)
                        self.robot_position_updated.emit(px, py)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"로봇 위치 처리 중 오류 발생: {e}")

        sock.close()
        print("로봇 위치 수신 스레드 종료.")

    def stop(self):
        """스레드를 안전하게 종료시킵니다."""
        self.is_running = False