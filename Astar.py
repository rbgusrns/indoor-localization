# pathfinder.py

import heapq
from collections import deque

class Node:
    """
    A* 알고리즘에 사용될 노드 클래스입니다.
    각 노드는 지도 위의 한 칸을 나타내며, 경로 탐색에 필요한 비용 정보를 가집니다.

    Attributes:
        parent (Node): 이 노드로 오기 직전의 부모 노드. 경로 역추적에 사용됩니다.
        position (tuple): 노드의 지도상 위치 (행, 열).
        g (int): 시작 노드로부터 현재 노드까지의 실제 이동 비용.
        h (int): 현재 노드로부터 목표 노드까지의 예상 비용 (휴리스틱).
        f (int): 총 예상 비용 (g + h).
    """
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        """두 노드의 위치가 같으면 동일한 노드로 취급합니다."""
        return self.position == other.position

    def __lt__(self, other):
        """
        heapq에서 노드들을 f값 기준으로 정렬하기 위한 비교 연산자입니다.
        f값이 같을 경우 h값이 더 작은 쪽을 우선합니다.
        """
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f


def create_distance_map(grid):

    global distance_map,max_distance,penalty_strength
    """
    BFS를 사용하여 모든 길(0) 타일에서 가장 가까운 벽(1)까지의 거리를 계산합니다.
    """
    rows, cols = len(grid), len(grid[0])
    distance_map = [[float('inf')] * cols for _ in range(rows)]
    queue = deque()

    # 1. 모든 벽을 큐에 추가하고 거리를 0으로 설정
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                distance_map[r][c] = 0
                queue.append((r, c))

    # 2. BFS 실행
    while queue:
        r, c = queue.popleft()
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and distance_map[nr][nc] == float('inf'):
                distance_map[nr][nc] = distance_map[r][c] + 1
                queue.append((nr, nc))
    
    # 3. 가장 먼 거리(가장 중앙) 값 찾기
    max_dist = 0
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 0:
                max_dist = max(max_dist, distance_map[r][c])

    return distance_map, max_dist


def find_path(grid, start, end):

    global distance_map,max_distance,penalty_strength
    """
    A* 알고리즘 (디버깅 버전)
    """
    if not grid or grid[start[0]][start[1]] != 0 or grid[end[0]][end[1]] != 0:
        return None

    distance_map = create_distance_map(grid)
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = []
    open_dict = {} 
    closed_set = set()

    heapq.heappush(open_list, start_node)
    open_dict[start_node.position] = start_node

    max_row = len(grid) - 1
    max_col = len(grid[0]) - 1

    # --- 🕵️ 디버깅 코드 추가 ---
    loop_count = 0
    
    while open_list:
        # --- 🕵️ 디버깅 코드 추가 ---
        loop_count += 1
        if loop_count > 1000:  # 맵 크기에 비해 비정상적으로 반복문이 많이 돈다면
            print("🚨 오류: 반복 횟수가 1000회를 초과했습니다. 무한 루프일 가능성이 높습니다.")
            return None # 강제 종료

        current_node = heapq.heappop(open_list)
        
        if current_node.position in closed_set:
            continue
            
        closed_set.add(current_node.position)
        
        if current_node == end_node:
            # --- 🕵️ 디버깅 코드 추가 ---
            print(f"✅ 경로 찾기 성공! (총 반복 횟수: {loop_count})")
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
            dist_to_wall = distance_map[node_position[0]][node_position[1]]
            
            # 2. 거리에 반비례하는 페널티 계산 (0~1 사이 값)
            # 벽에 가까울수록(dist_to_wall가 작을수록) 페널티가 1에 가까워짐
            penalty = (max_distance - dist_to_wall) / max_distance
            
            # 3. 최종 이동 비용 계산
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
    # 테스트용 가상 지도 (0: 길, 1: 벽)
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

    print(f"시작점: {start_point}, 도착점: {end_point}")
    path = find_path(test_map, start_point, end_point)

    if path:
        print(f"찾은 경로 (총 {len(path)} 단계):")
        print(path)

        # 경로를 지도에 시각적으로 표시
        for r, c in path:
            if (r, c) != start_point and (r, c) != end_point:
                test_map[r][c] = '*' # '*'는 경로를 의미

        print("\n경로 시각화:")
        for row in test_map:
            print(" ".join(str(cell) for cell in row))
    else:
        print("경로를 찾지 못했습니다.")