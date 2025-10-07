import json
import pandas as pd
import numpy as np
import lightgbm as lgb
import joblib
import warnings

from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split

warnings.filterwarnings('ignore')

class LGBM_Classifier_Predictor:
    def __init__(self):
        self.model = None
        ## [수정] feature_columns만 있으면 충분하므로 beacon_columns는 제거합니다.
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
        ## [설명] MAC 주소의 ':' 문자를 '_'로 변경하여 컬럼명으로 사용하기 쉽게 만듭니다.
        df.rename(columns=lambda c: c.replace(':', '_'), inplace=True)

        df['pos_label'] = df['x'].astype(str) + '_' + df['y'].astype(str)
        
        beacon_columns = [col for col in df.columns if col not in ['x', 'y', 'direction', 'pos_label']]
        df[beacon_columns] = df[beacon_columns].fillna(-100)
        
        # 'direction' 컬럼을 원-핫 인코딩으로 변환합니다.
        df = pd.get_dummies(df, columns=['direction'], prefix='dir')
        
        X = df.drop(['x', 'y', 'pos_label'], axis=1)
        y = df['pos_label']
        
        return X, y

    def train(self, db_path="fingerprint_db_4dir.json", test_size=0.3):
        """데이터를 불러와 LightGBM 분류 모델을 학습하고 정확도를 평가합니다."""
        X, y = self._prepare_data(db_path)
        if X is None:
            return

        # 학습에 사용된 최종 피처 컬럼들을 저장합니다. (원-핫 인코딩 포함)
        self.feature_columns = X.columns.tolist()

        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=test_size, random_state=42, stratify=y
        )

        print("분류 모델 학습을 시작합니다...")
        
        self.model = lgb.LGBMClassifier(objective='multiclass', n_estimators=200, random_state=42)
        self.model.fit(X_train, y_train)
        print("위치 분류 모델 학습 완료.")
        
        y_pred = self.model.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        print(f"✅ 모델 검증 정확도: {accuracy:.4f}")

    def predict(self, live_rssi_vector):
        """실시간 RSSI 벡터를 입력받아 위치 레이블(예: '2_2')을 예측합니다."""
        if self.model is None or self.feature_columns is None:
            print("오류: 모델이 학습되지 않았습니다. train() 또는 load_model()을 먼저 호출하세요.")
            return None

        # 1. 실시간 데이터를 DataFrame으로 변환하고 컬럼명을 학습 데이터와 맞게 수정
        sanitized_live_data = {k.replace(':', '_'): v for k, v in live_rssi_vector.items()}
        live_df = pd.DataFrame([sanitized_live_data])

        ## [수정] Pandas의 get_dummies와 reindex를 함께 사용하여 전처리 과정을 자동화하고 단순화합니다.
        # 2. 'direction' 컬럼을 원-핫 인코딩 처리
        live_df = pd.get_dummies(live_df)

        # 3. 학습된 전체 피처 컬럼 순서에 맞게 DataFrame을 재구성합니다.
        #    - live_df에 없는 컬럼은 새로 추가되고 fill_value로 채워집니다. (예: 잡히지 않은 비콘, 다른 방향)
        #    - 학습 시 없었던 컬럼은 자동으로 제거됩니다.
        live_df_aligned = live_df.reindex(columns=self.feature_columns, fill_value=-100)
        
        # 4. dir_ 컬럼들의 fill_value가 -100이 아닌 0이 되도록 수정
        dir_cols = [col for col in self.feature_columns if col.startswith('dir_')]
        live_df_aligned[dir_cols] = live_df_aligned[dir_cols].replace(-100, 0)

        # 5. 예측 수행
        predicted_label = self.model.predict(live_df_aligned)[0]

        return predicted_label

    def save_model(self, path="lgbm_predictor.pkl"):
        """학습된 모델과 피처 정보를 파일에 저장합니다."""
        if self.model is None:
            print("오류: 저장할 모델이 없습니다.")
            return
        
        ## [수정] 꼭 필요한 정보(모델, 피처 컬럼)만 저장하도록 단순화
        model_data = {
            'model': self.model,
            'feature_columns': self.feature_columns
        }
        joblib.dump(model_data, path)
        print(f"✅ 모델이 '{path}' 파일로 저장되었습니다.")

    def load_model(self, path="lgbm_predictor.pkl"):
        """파일에서 모델과 피처 정보를 불러옵니다."""
        try:
            model_data = joblib.load(path)
            self.model = model_data['model']
            self.feature_columns = model_data['feature_columns']
            print(f"✅ '{path}' 파일에서 모델을 성공적으로 불러왔습니다.")
            return True
        except FileNotFoundError:
            print(f"오류: '{path}' 파일을 찾을 수 없습니다. train()을 먼저 실행하세요.")
            return False

if __name__ == '__main__':
    # --- 1. 모델 학습 후 저장 (최초 한 번만 실행) ---
    print("--- 모델 학습 및 저장 단계 ---")
    predictor_trainer = LGBM_Classifier_Predictor()
    predictor_trainer.train()
    predictor_trainer.save_model()
    print("-" * 30)


    # --- 2. 저장된 모델 불러와서 예측 (실제 사용할 때) ---
    print("\n--- 저장된 모델 로드 및 예측 단계 ---")
    predictor_user = LGBM_Classifier_Predictor()
    is_loaded = predictor_user.load_model()

    if is_loaded:
        live_data = {
            'direction': 'W',
            'C3:00:00:44:DC:1A': -67, 'C3:00:00:44:DC:1B': -60,
            'C3:00:00:44:DC:1C': -61, 'C3:00:00:44:DC:1D': -70,
            'C3:00:00:44:DC:1E': -62, 'C3:00:00:44:DC:1F': -57
            # 학습 데이터에 있었지만 여기엔 없는 'C3:00:00:44:DC:1F' 같은 비콘은
            # predict 메소드 내에서 자동으로 -100으로 처리됩니다.
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