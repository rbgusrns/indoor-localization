import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import os

# 파일 로드
with open("fingerprint_db.json", "r") as f:
    data = json.load(f)

# 각 위치에서 비콘별 RSSI 수집
records = []
for entry in data:
    x, y = entry["pos"]
    for mac, rssi in entry["rssi"].items():
        records.append({"x": x, "y": y, "beacon": mac, "rssi": rssi})

df = pd.DataFrame(records)

# 평균 RSSI로 그룹화
df_avg = df.groupby(["x", "y", "beacon"]).mean().reset_index()

# 히트맵 저장 폴더 생성
os.makedirs("heatmaps", exist_ok=True)

# 비콘별 히트맵 생성
for beacon in df_avg["beacon"].unique():
    # 피벗 테이블 생성
    pivot_table = df_avg[df_avg["beacon"] == beacon].pivot_table(index="y", columns="x", values="rssi")

    # y=0이 위로 가게: index를 y 내림차순 정렬
    pivot_table = pivot_table.sort_index(ascending=False)

    plt.figure(figsize=(8, 6))
    ax = sns.heatmap(pivot_table, annot=False, cmap="coolwarm", cbar_kws={'label': 'RSSI'})

    plt.title(f"Heatmap for Beacon {beacon}")
    plt.xlabel("x")
    plt.ylabel("y")

    # 숫자 눈금도 아래로 증가하도록 시각적으로 맞춰줌
    ax.invert_yaxis()

    plt.tight_layout()
    plt.savefig(f"heatmaps/heatmap_{beacon.replace(':', '_')}.png")
    plt.close()


print("히트맵 이미지들이 'heatmaps' 폴더에 저장되었습니다.")
input("Press Enter to exit...")
