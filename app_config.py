#yaml 파일을 불러오고 해석하는 파일.

import yaml

def load_config(path="config.yaml"):  
    with open(path, 'r') as f:  
        return yaml.safe_load(f)

# 기본 예시 내용 (config.yaml)
# px_per_m: 20
# scan_interval: 1.0
# filter_window: 5
# k_neighbors: 3
# beacon_macs:
#   - "C3:00:00:1C:6C:A0"
#   - "C3:00:00:1C:6C:FE"
#   - "48:87:2D:7A:0D:E6"
#   - "48:87:2D:7A:0D:DE"
