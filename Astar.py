# pathfinder.py

import heapq

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

def find_path(grid, start, end):
    """
    A* 알고리즘을 사용하여 주어진 그리드에서 최적 경로를 찾습니다.

    Args:
        grid (list of list of int): 0(이동 가능)과 1(벽)으로 이루어진 2차원 지도.
        start (tuple): 시작 좌표 (행, 열).
        end (tuple): 도착 좌표 (행, 열).

    Returns:
        list of tuples: 시작점부터 도착점까지의 경로 좌표 리스트.
                        경로를 찾지 못한 경우 None을 반환합니다.
    """
    if not grid or grid[start[0]][start[1]] != 0 or grid[end[0]][end[1]] != 0:
        print("경고: 시작점 또는 도착점이 벽이거나 유효하지 않은 그리드입니다.")
        return None

    # 시작 노드와 도착 노드 초기화
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = []
    closed_set = set()

    # open_list를 min-heap으로 사용하여 항상 f값이 가장 작은 노드를 효율적으로 찾음
    heapq.heappush(open_list, start_node)

    # 지도의 가로, 세로 크기
    max_row = len(grid) - 1
    max_col = len(grid[0]) - 1

    while open_list:
        # f값이 가장 작은 노드를 현재 노드로 선택
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)

        # 목표 지점에 도달했으면 경로를 역추적하여 반환
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
            return path[::-1]  # 경로를 뒤집어서 시작점부터의 순서로 만듦

        # 인접한 노드(상, 하, 좌, 우) 탐색
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            node_position = (current_node.position[0] + new_position[0],
                             current_node.position[1] + new_position[1])

            # 1. 맵 범위를 벗어나는지 확인
            if not (0 <= node_position[0] <= max_row and 0 <= node_position[1] <= max_col):
                continue
            # 2. 벽인지 확인
            if grid[node_position[0]][node_position[1]] != 0:
                continue
            # 3. 이미 탐색이 끝난 노드인지 확인
            if node_position in closed_set:
                continue

            # 새로운 이웃 노드 생성
            neighbor = Node(current_node, node_position)

            # 비용(g, h, f) 계산
            neighbor.g = current_node.g + 1
            # 휴리스틱: 맨해튼 거리 사용 (가로, 세로 이동 거리의 합)
            neighbor.h = abs(neighbor.position[0] - end_node.position[0]) + \
                         abs(neighbor.position[1] - end_node.position[1])
            neighbor.f = neighbor.g + neighbor.h
            
            # 4. open_list에 이미 있는 노드이며, 새로운 경로가 더 비효율적인지 확인
            is_in_open_list = False
            for open_node in open_list:
                if neighbor == open_node and neighbor.g >= open_node.g:
                    is_in_open_list = True
                    break
            
            if not is_in_open_list:
                heapq.heappush(open_list, neighbor)

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