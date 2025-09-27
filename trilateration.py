# 칼만필터.
import math
from scipy.optimize import least_squares
from collections import deque
import numpy as np

class SuperFilter: #칼만필터 + 이동평균 필터

    def __init__(self, process_noise=0.08, measurement_noise=7, window_size=5):
        # Kalman 관련
        self.initialized = False
        self.processNoise = process_noise
        self.measurementNoise = measurement_noise
        self.predictedRSSI = 0.0
        self.errorCovariance = 0.0
        self.prev_rssi = 0.0
        self.recent_diffs = []

        # Moving Average 관련
        self.window = deque(maxlen=window_size)

    def kalman_filter(self, rssi):
        if not self.initialized:
            self.initialized = True
            prior_rssi = rssi
            prior_error_covariance = 1.0
            self.prev_rssi = rssi
        else:
            prior_rssi = self.predictedRSSI
            prior_error_covariance = self.errorCovariance + self.processNoise

            # 튐 감지
            diff = abs(rssi - self.prev_rssi)
            self.recent_diffs.append(diff)
            if len(self.recent_diffs) > 5:
                self.recent_diffs.pop(0)

            diff_mean = sum(self.recent_diffs) / len(self.recent_diffs)
            threshold = 6
            small_value = 2

            if diff > threshold and diff_mean < small_value:
                self.measurementNoise = 20
            else:
                self.measurementNoise = 7

            self.prev_rssi = rssi

        kalman_gain = prior_error_covariance / (prior_error_covariance + self.measurementNoise)
        self.predictedRSSI = prior_rssi + (kalman_gain * (rssi - prior_rssi))
        self.errorCovariance = (1 - kalman_gain) * prior_error_covariance

        return self.predictedRSSI

    def filtering(self, rssi):
        # 1. 칼만 필터 먼저 적용
        kalmaned = self.kalman_filter(rssi)

        # 2. 이동평균 필터 적용
        self.window.append(kalmaned)
        smoothed = sum(self.window) / len(self.window)

        return smoothed

###############################################################################
# 칼만 필터 (RSSI 필터링)
###############################################################################
class KalmanFilter:
    """
    간단한 Kalman Filter 구현.
    측정된 RSSI 값의 노이즈를 줄이는 역할을 수행합니다.
    """
    def __init__(self, process_noise=0.08, measurement_noise=7):
        self.initialized = False
        self.processNoise = process_noise
        self.measurementNoise = measurement_noise
        self.predictedRSSI = 0.0
        self.errorCovariance = 0.0

    def filtering(self, rssi):
        # === 초기화 ===
        if not self.initialized:
            self.initialized = True
            prior_rssi = rssi
            prior_error_covariance = 1.0
            self.prev_rssi = rssi
            self.recent_diffs = []  # 최근 diff 기록용
        else:
            prior_rssi = self.predictedRSSI
            prior_error_covariance = self.errorCovariance + self.processNoise

            # === 여기가 핵심 추가 ===
            diff = abs(rssi - self.prev_rssi)
            self.recent_diffs.append(diff)
            if len(self.recent_diffs) > 5:
                self.recent_diffs.pop(0)  # 최근 5개만 유지

            diff_mean = sum(self.recent_diffs) / len(self.recent_diffs)

            threshold = 6  # dBm 튐 기준 (ex: 6dBm 이상 튀면 튐으로 간주)
            small_value = 2  # 최근 변화가 2dBm 이하였으면 조용한 상황으로 간주

            if diff > threshold and diff_mean < small_value:
                self.measurementNoise = 20  # noise 크게 잡아서 필터 세게
            else:
                self.measurementNoise = 7  # 기본값

            self.prev_rssi = rssi  # 업데이트

    # === 칼만 필터 계산 ===
        kalman_gain = prior_error_covariance / (prior_error_covariance + self.measurementNoise)
        self.predictedRSSI = prior_rssi + (kalman_gain * (rssi - prior_rssi))
        self.errorCovariance = (1 - kalman_gain) * prior_error_covariance

        return self.predictedRSSI



import numpy as np

class EKF:
    def __init__(self, dt):
        self.dt = dt
        self.n = 4  # 상태: [px, py, theta(rad), v]
        self.m = 2  # 측정: [x_ble, y_ble]

        self.x = np.zeros(self.n)
        self.x[0] = 3.6  # 초기 x 위치
        self.x[1] = 3
        self.x[2] = 0.0  # 초기 yaw
        self.P = np.eye(self.n) * 0.1
        self.Q = np.diag([0.05, 0.05, 0.01, 0.1])

        # --- 수정된 부분 ①: 측정 노이즈 공분산(R) 값 증가 ---
        # 비콘의 정확도가 낮다는 것을 필터에 알려주기 위해 R 값을 높게 설정합니다.
        # R 값이 클수록 필터는 비콘 측정값(z)보다 내부 예측 모델(predict)을 더 신뢰하게 됩니다.
        # 예를 들어, 비콘 오차가 표준편차 2m 정도라고 가정하면, 분산은 2^2=4가 됩니다.
        # 기존: self.R = np.diag([0.2, 0.2])
        self.R = np.diag([4.0, 4.0])  # 표준편차 2m에 해당하는 분산 값으로 설정 (환경에 맞게 조절)

    def predict(self, imu_yaw, imu_speed):
        self.x[2] = np.deg2rad(imu_yaw)
        self.x[3] = imu_speed * 0.65

        theta = self.x[2]
        v = self.x[3]
        dt = self.dt

        self.x[0] += v * np.cos(theta) * dt
        self.x[1] += v * np.sin(theta) * dt

        A = np.eye(self.n)
        A[0, 2] = -v * np.sin(theta) * dt
        A[0, 3] = np.cos(theta) * dt
        A[1, 2] = v * np.cos(theta) * dt
        A[1, 3] = np.sin(theta) * dt

        self.P = A @ self.P @ A.T + self.Q

    def update(self, z):
        # z: [x_ble, y_ble]
        z_pred = self.x[:2]
        H = np.zeros((2, self.n))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        y = z - z_pred
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # --- 수정된 부분 ②: 상태 업데이트 크기 제한 ---
        # 칼만 이득(K)과 측정 오차(y)를 통해 계산된 보정량을 구합니다.
        update_vector = K @ y

        # 위치에 대한 보정량 [dx, dy]를 추출합니다.
        position_update = update_vector[:2]
        
        # 보정량의 크기(이동 거리)를 계산합니다.
        update_distance = np.linalg.norm(position_update)
        
        # 최대 허용 거리(20cm)를 정의합니다.
        max_update_distance = 0.01

        # 만약 계산된 보정 거리가 20cm를 초과하면
        if update_distance > max_update_distance:
            # 보정 벡터의 방향은 유지하되, 크기를 20cm로 줄입니다.
            scale_factor = max_update_distance / update_distance
            update_vector = update_vector * scale_factor

        # 크기가 조절된 보정량을 최종 상태에 적용합니다.
        self.x = self.x + update_vector
        # ---------------------------------------------
        
        self.P = (np.eye(self.n) - K @ H) @ self.P

    def get_state(self):
        return self.x.copy()