import json
import pandas as pd
import numpy as np  # NumPy를 임포트합니다.
import lightgbm as lgb
import joblib     # 모델 저장/불러오기를 위해 추가합니다.
import warnings

# 모델 성능 평가를 위해 추가합니다.
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split

warnings.filterwarnings('ignore')

class LGBM_Classifier_Predictor:
    def __init__(self):
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

        df['pos_label'] = df['x'].astype(str) + '_' + df['y'].astype(str)
        
        self.beacon_columns = [col for col in df.columns if col not in ['x', 'y', 'direction', 'pos_label']]
        df[self.beacon_columns] = df[self.beacon_columns].fillna(-100)
        df = pd.get_dummies(df, columns=['direction'], prefix='dir')
        
        X = df.drop(['x', 'y', 'pos_label'], axis=1)
        y = df['pos_label']
        
        return X, y

    def train(self, db_path="fingerprint_db_4dir.json", test_size=0.2):
        """데이터를 불러와 LightGBM 분류 모델을 학습하고 정확도를 평가합니다."""
        X, y = self._prepare_data(db_path)
        if X is None:
            return

        # 데이터를 학습용과 검증용으로 분리합니다.
        # stratify=y 옵션은 각 위치 레이블의 비율을 유지하며 분리해 안정성을 높입니다.
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=test_size, random_state=42, stratify=y
        )

        print("분류 모델 학습을 시작합니다...")
        
        self.model = lgb.LGBMClassifier(objective='multiclass', n_estimators=200, random_state=42)
        self.model.fit(X_train, y_train)
        print("위치 분류 모델 학습 완료.")
        
        # 검증용 데이터로 예측 및 정확도를 평가하여 모델 성능을 확인합니다.
        y_pred = self.model.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        print(f"✅ 모델 검증 정확도: {accuracy:.4f}")
        
        # 모델이 예측 시 사용할 최종 피처 컬럼 순서를 저장합니다.
        self.feature_columns = X.columns

    def predict(self, live_rssi_vector):
        """실시간 RSSI 벡터를 입력받아 위치 레이블(예: '2_2')을 예측합니다."""
        if self.model is None or self.feature_columns is None:
            print("오류: 모델이 학습되지 않았습니다. train() 또는 load_model()을 먼저 호출하세요.")
            return None

        # 1. 입력된 딕셔너리를 DataFrame으로 변환
        sanitized_live_data = {k.replace(':', '_'): v for k, v in live_rssi_vector.items()}
        live_df = pd.DataFrame([sanitized_live_data])

        # 2. 학습된 컬럼 순서에 맞게 DataFrame을 재구성 (reindex)
        #    - 학습 시 없었던 비콘/방향 정보가 들어와도 무시되고,
        #    - 학습 시 있었지만 지금은 없는 정보는 fill_value로 채워집니다.
        live_df_aligned = live_df.reindex(columns=self.feature_columns, fill_value=0)
        
        # 3. 누락된 비콘 신호의 기본값(-100)을 다시 채워줍니다.
        for col in self.beacon_columns:
            if col not in sanitized_live_data:
                live_df_aligned[col] = -100

        # ▼▼▼ [오류 해결] DataFrame을 NumPy 배열로 변환 ▼▼▼
        # 모델은 .shape 속성이 있는 NumPy 배열을 기대하므로, 예측 직전에 변환합니다.
        live_array = live_df_aligned.to_numpy()
        
        # 변환된 NumPy 배열로 예측을 수행합니다.
        predicted_label = self.model.predict(live_array)[0]

        return predicted_label

    def save_model(self, path="lgbm_predictor.pkl"):
        """학습된 모델과 피처 정보를 파일에 저장합니다."""
        if self.model is None:
            print("오류: 저장할 모델이 없습니다.")
            return
        
        model_data = {
            'model': self.model,
            'beacon_columns': self.beacon_columns,
            'feature_columns': self.feature_columns
        }
        joblib.dump(model_data, path)
        print(f"✅ 모델이 '{path}' 파일로 저장되었습니다.")

    def load_model(self, path="lgbm_predictor.pkl"):
        """파일에서 모델과 피처 정보를 불러옵니다."""
        try:
            model_data = joblib.load(path)
            self.model = model_data['model']
            self.beacon_columns = model_data['beacon_columns']
            self.feature_columns = model_data['feature_columns']
            print(f"✅ '{path}' 파일에서 모델을 성공적으로 불러왔습니다.")
            return True
        except FileNotFoundError:
            print(f"오류: '{path}' 파일을 찾을 수 없습니다. train()을 먼저 실행하세요.")
            return False

# --- 사용 예시 ---
if __name__ == '__main__':
    # --- 1. 모델 학습 후 저장 (최초 한 번만 실행) ---
    print("--- 모델 학습 및 저장 단계 ---")
    predictor_trainer = LGBM_Classifier_Predictor()
    predictor_trainer.train()
    predictor_trainer.save_model() # 학습된 모델을 파일로 저장
    print("-" * 30)


    # --- 2. 저장된 모델 불러와서 예측 (실제 사용할 때) ---
    print("\n--- 저장된 모델 로드 및 예측 단계 ---")
    predictor_user = LGBM_Classifier_Predictor()
    is_loaded = predictor_user.load_model() # 파일에서 모델 로드

    if is_loaded:
        live_data = {
            'direction': 'W',
            'C3:00:00:44:DC:1A': -67, 'C3:00:00:44:DC:1B': -60,
            'C3:00:00:44:DC:1C': -61, 'C3:00:00:44:DC:1D': -70,
            'C3:00:00:44:DC:1E': -62, 'C3:00:00:44:DC:1F': -57
        }
        
        predicted_pos_label = predictor_user.predict(live_data)
        
        if predicted_pos_label:
            try:
                pred_x, pred_y = map(int, predicted_pos_label.split('_'))
                print(f"\n입력된 RSSI 데이터: {live_data}")
                print(f"예측된 그리드: '{predicted_pos_label}'")
                print(f"🎯 최종 좌표: ({pred_x}, {pred_y})")
            except ValueError:
                print(f"오류: 예측된 레이블 '{predicted_pos_label}'을 좌표로 변환할 수 없습니다.")