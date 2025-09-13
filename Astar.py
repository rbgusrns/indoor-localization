import heapq
from collections import deque

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f

def create_distance_map(grid):
    """
    BFS를 사용하여 모든 길(0) 타일에서 가장 가까운 벽(1)까지의 거리를 계산합니다.
    """
    rows, cols = len(grid), len(grid[0])
    distance_map = [[float('inf')] * cols for _ in range(rows)]
    queue = deque()

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                distance_map[r][c] = 0
                queue.append((r, c))

    while queue:
        r, c = queue.popleft()
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and distance_map[nr][nc] == float('inf'):
                distance_map[nr][nc] = distance_map[r][c] + 1
                queue.append((nr, nc))
    
    max_dist = 0
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 0:
                max_dist = max(max_dist, distance_map[r][c])

    return distance_map, max_dist

def find_path(grid, start, end, distance_map, max_dist, penalty_strength):
    """
    A* 알고리즘 (중앙 경로 선호 로직 적용)
    """
    if not grid or grid[start[0]][start[1]] != 0 or grid[end[0]][end[1]] != 0:
        return None

    start_node = Node(None, start)
    end_node = Node(None, end)
    open_list = []
    open_dict = {} 
    closed_set = set()

    heapq.heappush(open_list, start_node)
    open_dict[start_node.position] = start_node

    max_row = len(grid) - 1
    max_col = len(grid[0]) - 1
    
    while open_list:
        current_node = heapq.heappop(open_list)
        
        if current_node.position in closed_set:
            continue
            
        closed_set.add(current_node.position)
        
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            node_position = (current_node.position[0] + new_position[0],
                             current_node.position[1] + new_position[1])

            if not (0 <= node_position[0] <= max_row and 0 <= node_position[1] <= max_col):
                continue
            if grid[node_position[0]][node_position[1]] != 0:
                continue
            if node_position in closed_set:
                continue

            neighbor = Node(current_node, node_position)
            
            # --- 중앙 경로 선호를 위한 비용 계산 ---
            dist_to_wall = distance_map[node_position[0]][node_position[1]]
            penalty = (max_dist - dist_to_wall) / max_dist if max_dist > 0 else 0
            step_cost = 1 + penalty_strength * penalty
            
            neighbor.g = current_node.g + step_cost
            neighbor.h = abs(neighbor.position[0] - end_node.position[0]) + \
                         abs(neighbor.position[1] - end_node.position[1])
            neighbor.f = neighbor.g + neighbor.h
            
            if node_position in open_dict and open_dict[node_position].g <= neighbor.g:
                continue
            
            heapq.heappush(open_list, neighbor)
            open_dict[node_position] = neighbor

    print("경로를 찾을 수 없습니다.")
    return None

# --- 사용 예시 ---
if __name__ == '__main__':
    test_map = [
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 1, 1, 1, 0]
    ]
    start_point = (0, 0)
    end_point = (8, 9)

    # 1. 경로 탐색 전, 맵을 분석하여 거리 맵을 미리 생성
    distance_map, max_dist = create_distance_map(test_map)
    
    # 2. 페널티 강도 설정 (값이 클수록 중앙을 더 강하게 선호)
    penalty_strength = 10.0

    print(f"시작점: {start_point}, 도착점: {end_point}")
    # 3. find_path 함수에 필요한 모든 정보를 인자로 전달
    path = find_path(test_map, start_point, end_point, distance_map, max_dist, penalty_strength)

    if path:
        print(f"찾은 경로 (총 {len(path)} 단계):")
        print(path)
        
        map_for_display = [row[:] for row in test_map]
        for r, c in path:
            if (r, c) != start_point and (r, c) != end_point:
                map_for_display[r][c] = '*'

        print("\n경로 시각화:")
        for row in map_for_display:
            print(" ".join(str(cell) for cell in row))
    else:
        print("경로를 찾지 못했습니다.")