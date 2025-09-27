# train_model.py (수정)
import joblib
from lgbm_predictor import LGBM_Classifier_Predictor

if __name__ == '__main__':
    predictor = LGBM_Classifier_Predictor()
    predictor.train("fingerprint_db_4dir.json")
    
    # [수정] 모델 객체 대신 predictor 인스턴스 전체를 저장
    joblib.dump(predictor, 'lgbm_predictor.pkl')
    print("학습된 Predictor 객체가 'lgbm_predictor.pkl' 파일로 저장되었습니다.")