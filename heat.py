import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import os

# --- 실행 전, 파일 이름을 4방향 데이터가 저장된 파일명으로 변경해주세요 ---
FINGERPRINT_DB_FILE = "fingerprint_db_4dir.json"

# 파일 로드
try:
    with open(FINGERPRINT_DB_FILE, "r") as f:
        data = json.load(f)
except FileNotFoundError:
    print(f"오류: '{FINGERPRINT_DB_FILE}' 파일을 찾을 수 없습니다.")
    input("Press Enter to exit...")
    sys.exit(1)


# --- 변경된 부분: 방향(direction) 정보 추가 ---
# 각 위치 및 방향에서 비콘별 RSSI 수집
records = []
for entry in data:
    # 키가 'pos'가 아닌 경우 건너뛰기 (데이터 형식 오류 방지)
    if 'pos' not in entry or len(entry['pos']) != 3:
        continue
    
    x, y, direction = entry["pos"]
    for mac, rssi in entry["rssi"].items():
        records.append({"x": x, "y": y, "direction": direction, "beacon": mac, "rssi": rssi})

df = pd.DataFrame(records)

# --- 변경된 부분: direction을 기준으로 추가하여 그룹화 ---
# 평균 RSSI로 그룹화
df_avg = df.groupby(["x", "y", "direction", "beacon"]).mean().reset_index()

# 히트맵 저장 메인 폴더 생성
os.makedirs("heatmaps", exist_ok=True)

# --- 변경된 부분: 방향별, 비콘별 히트맵을 생성하는 이중 반복문 ---
# 1. 방향별로 반복
for direction in df_avg["direction"].unique():
    print(f"Processing direction: {direction}...")
    
    # 방향별 하위 폴더 생성
    dir_path = os.path.join("heatmaps", direction)
    os.makedirs(dir_path, exist_ok=True)
    
    # 2. 해당 방향 내에서 비콘별로 반복
    for beacon in df_avg["beacon"].unique():
        # 현재 방향과 비콘에 해당하는 데이터만 필터링
        beacon_dir_df = df_avg[(df_avg["direction"] == direction) & (df_avg["beacon"] == beacon)]
        
        # 데이터가 없는 경우 히트맵 생성을 건너뜀
        if beacon_dir_df.empty:
            continue
            
        # 피벗 테이블 생성
        pivot_table = beacon_dir_df.pivot_table(index="y", columns="x", values="rssi")

        # y=0이 위로 가게: index를 y 내림차순 정렬
        pivot_table = pivot_table.sort_index(ascending=False)

        plt.figure(figsize=(8, 6))
        ax = sns.heatmap(pivot_table, annot=False, cmap="coolwarm", cbar_kws={'label': 'RSSI (dBm)'},
                         vmin=-90, vmax=-40) # RSSI 값의 범위를 지정하여 모든 히트맵의 색상 스케일을 통일

        plt.title(f"Beacon: {beacon}\nDirection: {direction}")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")

        # 숫자 눈금도 아래로 증가하도록 시각적으로 맞춰줌
        ax.invert_yaxis()

        plt.tight_layout()
        # 저장 경로를 방향별 하위 폴더로 지정
        plt.savefig(os.path.join(dir_path, f"heatmap_{beacon.replace(':', '_')}.png"))
        plt.close()

print("\n히트맵 이미지들이 'heatmaps' 폴더 안에 방향별로 저장되었습니다.")
input("Press Enter to exit...")