class EKF:
    def __init__(self, dt, beacon_pos):
        self.dt = dt
        self.n = 4  # 상태 차원: [px, py, theta, v]
        self.m = 4  # 측정 차원: [r1, r2, r3, theta]
        self.x = np.zeros(self.n)  # 1*4 상태 행렬 > 0으로 초기화화
        self.P = np.eye(self.n) * 0.1  # 4*4 공분산 행렬 만듦 > 주대각성분 0.1로 초기화화
        self.Q = np.diag([0.05, 0.05, 0.01, 0.1])  # 프로세스 잡음 > 보류
        self.R = np.diag([0.3, 0.3, 0.3, 0.05])  # 측정 잡음 > 보류류
        self.beacons = np.array(beacon_pos)  # 비콘 위치 [ [bx, by], ... ]

    def predict(self, imu_yaw, imu_speed):
        
        # IMU 센서로부터 yaw와 속도 정보를 받아 상태 벡터 업데이트
        self.x[2] = np.deg2rad(imu_yaw)
        self.x[3] = imu_speed
        
        theta = self.x[2]
        v = self.x[3]
        dt = self.dt

        # 상태 예측 수식 적용
        self.x[0] += v * np.cos(theta) * dt
        self.x[1] += v * np.sin(theta) * dt


        # 자코비안 A 계산
        A = np.eye(self.n)
        A[0, 2] = -v * np.sin(theta) * dt
        A[0, 3] = np.cos(theta) * dt
        A[1, 2] = v * np.cos(theta) * dt
        A[1, 3] = np.sin(theta) * dt

        # 공분산 예측
        self.P = A @ self.P @ A.T + self.Q

    def update(self, z): # z는 [r1, r2, r3, theta] 형태의 측정값
        z_pred = np.zeros(self.m) #4*1 측정 예측 행렬 생성 및 초기화
        H = np.zeros((self.m, self.n)) #4*4 h행렬 생성 및 초기화

        # 각 비콘에 대해 거리 예측 및 자코비안 H 계산
        for i in range(3):
            dx = self.x[0] - self.beacons[i][0] 
            dy = self.x[1] - self.beacons[i][1]
            r = np.sqrt(dx**2 + dy**2)
            z_pred[i] = r
            if r < 1e-4: r = 1e-4
            H[i, 0] = dx / r
            H[i, 1] = dy / r

        # yaw 값 예측
        z_pred[3] = self.x[2] #theta
        H[3, 2] = 1.0

        # 측정 오차 / z - h(xk)
        y = z - z_pred

        # 칼만 필터 계산 / h*pk*hT + R(노이즈는 배제해도 될듯)
        S = H @ self.P @ H.T + self.R # R은 없애도 될듯
        K = self.P @ H.T @ np.linalg.inv(S) #칼만 이득 계산

        # 상태 업데이트 / 최종 추정값 계산 피드백
        self.x = self.x + K @ y

        # 공분산 업데이트 / 최종 공분산 계산 피드백
        I = np.eye(self.n)
        self.P = (I - K @ H) @ self.P

    def get_state(self):
        return self.x.copy()
