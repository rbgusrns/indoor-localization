
import json
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# 파일 로드
with open("fingerprint_db.json", "r") as f:
    data = json.load(f)

# 데이터프레임으로 변환
records = []
for entry in data:
    x = entry["x"]
    y = entry["y"]
    for beacon_id, rssi in entry["beacons"].items():
        records.append({"x": x, "y": y, "beacon": beacon_id, "rssi": rssi})

df = pd.DataFrame(records)

# 비콘별로 히트맵 생성
beacons = df["beacon"].unique()
for beacon in beacons:
    pivot_table = df[df["beacon"] == beacon].pivot_table(index="y", columns="x", values="rssi")
    plt.figure(figsize=(8, 6))
    sns.heatmap(pivot_table, annot=False, cmap="coolwarm", cbar_kws={'label': 'RSSI'})
    plt.title(f"Heatmap for {beacon}")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.gca().invert_yaxis()
    plt.show()
