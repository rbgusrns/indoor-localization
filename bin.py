from PIL import Image


def create_binary_map(image_path, block_size=10, wall_threshold=128):
    """
    이미지 파일을 입력받아 20x20 픽셀 블록 단위로 이진화된 지도 리스트를 생성합니다.

    Args:
        image_path (str): 지도 이미지 파일 경로
        block_size (int): 검사할 픽셀 묶음의 크기 (기본값: 20)
        wall_threshold (int): 벽으로 판단할 픽셀 밝기의 임계값 (0-255, 기본값: 128)

    Returns:
        list: 0 (길)과 1 (벽)으로 구성된 2차원 리스트
    """
    try:
        # 이미지를 회색조(grayscale)로 열기
        img = Image.open(image_path).convert('L')
        width, height = img.size

        binary_map = []
        # 이미지를 block_size 단위로 순회
        for y in range(0, height, block_size):
            row = []
            for x in range(0, width, block_size):
                # block_size 크기의 사각형 영역 잘라내기
                box = (x, y, x + block_size, y + block_size)
                block = img.crop(box)

                # 픽셀들의 평균 밝기 계산
                pixels = list(block.getdata())
                avg_brightness = sum(pixels) / len(pixels) if pixels else 0

                # 평균 밝기가 임계값보다 낮으면 벽(1), 아니면 길(0)로 처리
                if avg_brightness < wall_threshold:
                    row.append(1)  # 검은색에 가까우면 벽
                else:
                    row.append(0)  # 흰색에 가까우면 길
            binary_map.append(row)

        return binary_map

    except FileNotFoundError:
        print(f"오류: '{image_path}' 파일을 찾을 수 없습니다.")
        return None
    


# --- 사용 예시 ---
# 지도 이미지 파일 경로
map_image_file = 'map.png'  

# 함수 호출하여 이진 맵 생성
# 이미지 크기가 700x500이고 block_size가 20이면, 결과는 25x35 크기의 2차원 리스트가 됩니다.
binary_grid = create_binary_map(map_image_file)

# 결과 출력
if binary_grid:
    for row in binary_grid:
        # 각 숫자를 공백으로 구분하여 보기 좋게 출력
        print(' '.join(map(str, row)))


