import heapq
from collections import deque

class Node:
    """
    A* 알고리즘에 사용될 노드 클래스입니다.
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

def find_path(grid, start, end, distance_map, max_dist, penalty_strength):
    """
    A* 알고리즘 (대각선 이동 및 중앙 경로 선호 로직 적용)
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

        # --- ▼ 대각선 이동 기능 추가 ▼ ---
        # 8방향(상하좌우, 대각선)으로 탐색
        for move_r, move_c in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_position = (current_node.position[0] + move_r,
                             current_node.position[1] + move_c)

            if not (0 <= node_position[0] <= max_row and 0 <= node_position[1] <= max_col):
                continue
            if grid[node_position[0]][node_position[1]] != 0:
                continue
            if node_position in closed_set:
                continue

            # 대각선 이동 시 코너를 통과하지 못하도록 방지
            if abs(move_r) == 1 and abs(move_c) == 1:
                if grid[current_node.position[0] + move_r][current_node.position[1]] != 0 or \
                   grid[current_node.position[0]][current_node.position[1] + move_c] != 0:
                    continue

            neighbor = Node(current_node, node_position)
            
            # 이동 비용 계산 (직선: 1, 대각선: 1.414)
            base_cost = 1.414 if abs(move_r) == 1 and abs(move_c) == 1 else 1.0

            # 중앙 경로 선호를 위한 페널티 계산
            dist_to_wall = distance_map[node_position[0]][node_position[1]]
            penalty = (max_dist - dist_to_wall) / max_dist if max_dist > 0 else 0
            step_cost = base_cost + penalty_strength * penalty
            
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
