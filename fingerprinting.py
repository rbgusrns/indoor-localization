#핑거프린팅 알고리즘 파일.

import json
import numpy as np
from sklearn.neighbors import BallTree
from collections import defaultdict

class FingerprintDB:
    def __init__(self, grid_size=(1.0, 1.0), required_samples=100):
        self.records = []
        self.tree = None
        self.rssi_matrix = None
        self.normalized_matrix = None
        self.norms = None
        self.positions = None
        self.macs = []
        self.grid_size = grid_size
        self._acc_buffer = defaultdict(list) # defaultdict: 없는 키에 접근하면 자동으로 value 생성.
        self.required_samples = required_samples

    def _average_rssi(self, rssi_list):
        if not rssi_list:
            return {}
        macs = sorted({mac for r in rssi_list for mac in r})
        M = np.full((len(rssi_list), len(macs)), -100.0)
        for i, r in enumerate(rssi_list):
            for j, mac in enumerate(macs):
                if mac in r:
                    M[i, j] = r[mac]
        if M.shape[0] >= 3:
            mean = np.mean(M, axis=0)
            std = np.std(M, axis=0)
            mask = np.abs(M - mean) <= 2 * std
            M = np.where(mask, M, np.nan)
        col_mean = np.nanmean(M, axis=0)
        col_mean = np.where(np.isnan(col_mean), -100.0, col_mean)
        return {mac: float(col_mean[j]) for j, mac in enumerate(macs)}

    def collect(self, pos, rssi_vector):
        pos_key = tuple(pos) if isinstance(pos, (list, tuple)) else (pos,)
        
        # 1. 평균을 내지 않고, 들어온 데이터를 바로 records에 추가합니다.
        self.records.append({'pos': list(pos_key), 'rssi': rssi_vector.copy()})
        
        # 2. 샘플 개수를 세기 위한 로직은 그대로 유지합니다. (캘리브레이션 제어를 위해 필요)
        self._acc_buffer[pos_key].append(1) # 실제 데이터 대신 카운트용 숫자만 넣어도 됩니다.
        
        if len(self._acc_buffer[pos_key]) >= self.required_samples:
            # 3. 목표 개수에 도달하면 버퍼를 비우고 True를 반환합니다.
            self._acc_buffer[pos_key] = []
            return True
            
        return False

    def build_index(self):
        if not self.records:
            raise RuntimeError("No records to index")
        mac_set = set() # 중복된 mac 주소를 제거하기 위한 집합
        for rec in self.records:
            mac_set.update(rec['rssi'].keys()) # 딕셔너리의 모든 키 (mac 주소)를 추가.
        self.macs = sorted(mac_set)
        self.rssi_matrix = np.array(
            [[rec['rssi'].get(mac, -100) for mac in self.macs] for rec in self.records]
            
        )
        '''
        self.records = 
        [
            {'rssi': {'A': -70, 'B': -80}}, 
            {'rssi': {'B': -65, 'C': -90}}, 
        ] 
        위에서 아래로 저장.
        self.rssi_matrix = 
        [
            [-70, -80, -100],  # 첫 rec: A=-70, B=-80, C 없음 → -100
            [-100, -65, -90]   # 두 번째 rec: A 없음 → -100, B=-65, C=-90
        ]
        
        '''
        self.positions = np.array([rec['pos'] for rec in self.records])
        self.norms = np.linalg.norm(self.rssi_matrix, axis=1)
        norm_matrix = self.norms[:, np.newaxis]
        norm_matrix[norm_matrix == 0] = 1
        self.normalized_matrix = self.rssi_matrix / norm_matrix
        self.tree = BallTree(self.normalized_matrix, metric='euclidean')

    def save(self, path="fingerprint_db.json"):
        with open(path, 'w') as f:
            json.dump(self.records, f, indent=4, ensure_ascii=False)

    def load(self, path="fingerprint_db.json"):
        with open(path, 'r') as f:
            self.records = json.load(f)
        self.build_index()

    def get_position(self, rssi_vector, k=1, alpha=0.6, beta=0.8, strong_threshold=-70):
            """
            rssi_vector: dict MAC->RSSI
            k: top-k 후보 개수
            alpha: cosine vs norm_diff 비중 (0.0~1.0)
            beta: weighted L1 비중 (0.0~1.0)
            strong_threshold: '강신호'로 간주할 RSSI 임계치 (dBm)
            """
            if self.tree is None:
                raise RuntimeError("FingerprintDB not indexed. Call load() or build_index() first.")

            # 1) raw 샘플 벡터
            raw = np.array([rssi_vector.get(mac, -100) for mac in self.macs])
            norm_raw = np.linalg.norm(raw) or 1.0
            sample_normed = (raw / norm_raw).reshape(1, -1)

            # 2) cosine distance
            cos_dist, idx_all = self.tree.query(sample_normed, k=len(self.records))
            cos_dist = cos_dist[0]

            # 3) norm difference
            norm_diff = np.abs(self.norms - norm_raw)

            # 4) weighted L1 (강신호 AP 절댓값 차이)
            strong_mask = (raw > strong_threshold).astype(float)
            repeated = np.repeat(raw.reshape(1, -1), self.rssi_matrix.shape[0], axis=0)
            abs_diff = np.abs(self.rssi_matrix - repeated)
            weighted_l1 = (abs_diff * strong_mask).sum(axis=1)
            weighted_l1 /= max(strong_mask.sum(), 1)

            # 5) hybrid 거리
            hybrid = alpha * cos_dist + (1 - alpha) * norm_diff + beta * weighted_l1

            # 6) top-k + 가중평균
            topk = np.argsort(hybrid)[:k]
            pts = self.positions[topk]
            dists = hybrid[topk]
            weights = 1.0 / (dists + 1e-5)
            ble_pos = np.average(pts, axis=0, weights=weights)

            return ble_pos, pts, dists