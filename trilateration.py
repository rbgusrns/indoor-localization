# trilateration.py
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



class EKF:
    def __init__(self, dt):
        self.dt = dt
        self.n = 4  # 상태: [px, py, theta(rad), v]
        self.m = 2  # 측정: [x_ble, y_ble]

        self.x = np.zeros(self.n)
        self.x[0] = 1.2  # 초기 x 위치
        self.x[1] = 0.3
        self.x[2] = 0.0  # 초기 yaw
        self.P = np.eye(self.n) * 0.1
        self.Q = np.diag([0.05, 0.05, 0.01, 0.1])
        self.R = np.diag([0.2, 0.2])  # x, y만 사용

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
        A[1, 2] =  v * np.cos(theta) * dt
        A[1, 3] = np.sin(theta) * dt

        self.P = A @ self.P @ A.T + self.Q

    def update(self, z):
        # z: [x_ble, y_ble]
        z_pred = self.x[:2]  # 예측 위치
        H = np.zeros((2, self.n)) #관측 모델
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        y = z - z_pred #관측 오차
        S = H @ self.P @ H.T + self.R #예측 오차 공분산
        K = self.P @ H.T @ np.linalg.inv(S) # 칼만 이득

        self.x = self.x + K @ y # 상태 업데이트
        self.P = (np.eye(self.n) - K @ H) @ self.P # 공분산 업데이트

    def get_state(self):
        return self.x.copy()

