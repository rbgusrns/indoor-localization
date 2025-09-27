import json
import pandas as pd
import lightgbm as lgb
# [변경] 분류 모델의 성능 평가를 위해 추가 (선택 사항)
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split
import warnings

warnings.filterwarnings('ignore')

class LGBM_Classifier_Predictor: # [변경] 클래스 이름 변경
    def __init__(self):
        # [변경] x, y 모델을 하나로 통합
        self.model = None
        self.beacon_columns = None
        self.feature_columns = None

    def _prepare_data(self, db_path):
        """JSON 데이터를 불러와 피처와 '분류용 레이블'로 변환합니다."""
        try:
            with open(db_path, "r") as f:
                data = json.load(f)
        except FileNotFoundError:
            print(f"오류: '{db_path}' 파일을 찾을 수 없습니다.")
            return None, None

        records = []
        for entry in data:
            if 'pos' not in entry or len(entry['pos']) != 3:
                continue
            x, y, direction = entry["pos"]
            record = {'x': x, 'y': y, 'direction': direction}
            record.update(entry["rssi"])
            records.append(record)
        
        df = pd.DataFrame(records)
        df.rename(columns=lambda c: c.replace(':', '_'), inplace=True)

        # [변경] y 좌표를 숫자가 아닌, 'x_y' 형태의 문자열 레이블로 생성
        # 예: x=0, y=0 -> '0_0', x=2, y=2 -> '2_2'
        df['pos_label'] = df['x'].astype(str) + '_' + df['y'].astype(str)
        
        self.beacon_columns = [col for col in df.columns if col not in ['x', 'y', 'direction', 'pos_label']]
        df[self.beacon_columns] = df[self.beacon_columns].fillna(-100)
        df = pd.get_dummies(df, columns=['direction'], prefix='dir')
        
        # [변경] 피처에서 x, y, pos_label 모두 제거
        X = df.drop(['x', 'y', 'pos_label'], axis=1)
        # [변경] y는 새로 만든 pos_label을 사용
        y = df['pos_label']
        
        return X, y

    def train(self, db_path="fingerprint_db_4dir.json"):
        """데이터를 불러와 위치 레이블을 예측하는 LightGBM '분류' 모델을 학습합니다."""
        X, y = self._prepare_data(db_path)
        if X is None:
            return

        print("분류 모델 학습을 시작합니다...")
        
        # [변경] LGBMRegressor -> LGBMClassifier로 모델 변경
        self.model = lgb.LGBMClassifier(objective='multiclass', n_estimators=200, random_state=42)
        self.model.fit(X, y)
        print("위치 분류 모델 학습 완료.")
        
        self.feature_columns = X.columns

    def predict(self, live_rssi_vector):
        """실시간 RSSI 벡터를 입력받아 위치 레이블(예: '2_2')을 예측합니다."""
        if self.model is None:
            print("오류: 모델이 학습되지 않았습니다. 먼저 train() 메소드를 호출하세요.")
            return None

        sanitized_live_data = {k.replace(':', '_'): v for k, v in live_rssi_vector.items()}
        live_df = pd.DataFrame([sanitized_live_data])
        
        for col in self.beacon_columns:
            if col not in live_df:
                live_df[col] = -100
        
        live_df = pd.get_dummies(live_df, columns=['direction'], prefix='dir')
        live_df_aligned = live_df.reindex(columns=self.feature_columns, fill_value=0)
        
        # [변경] 모델이 위치 레이블을 직접 예측
        predicted_label = self.model.predict(live_df_aligned)[0]

        return predicted_label

# --- 사용 예시 ---
if __name__ == '__main__':
    predictor = LGBM_Classifier_Predictor()
    predictor.train()

    live_data = {
        'direction': 'W',
        'C3:00:00:44:DC:1A': -56,
        'C3:00:00:44:DC:1B': -58,
        'C3:00:00:44:DC:1C': -62,
        'C3:00:00:44:DC:1D': -69,
        'C3:00:00:44:DC:1E': -65,
        'C3:00:00:44:DC:1F': -65
    }
#{"pos": [7, 5, "W"], "rssi": {"C3:00:00:44:DC:1A": -67.76109545587136, "C3:00:00:44:DC:1B": -60.35010231583979, "C3:00:00:44:DC:1C": -61.69063421706986, "C3:00:00:44:DC:1D": -70.36670578272637, "C3:00:00:44:DC:1E": -62.61269748249295, "C3:00:00:44:DC:1F": -57.18151989980791}}]
    predicted_pos_label = predictor.predict(live_data)
    
    if predicted_pos_label:
        # [변경] 예측된 레이블('x_y')을 다시 숫자 좌표로 분리
        try:
            pred_x, pred_y = map(int, predicted_pos_label.split('_'))
            print(f"\n입력된 RSSI 데이터: {live_data}")
            print(f"예측된 그리드: {predicted_pos_label}")
            print(f"최종 좌표: ({pred_x}, {pred_y})")
        except ValueError:
            print(f"오류: 예측된 레이블 '{predicted_pos_label}'을 좌표로 변환할 수 없습니다.")