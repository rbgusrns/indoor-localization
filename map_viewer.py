from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsPixmapItem, QGraphicsPolygonItem, QGraphicsLineItem
from PyQt5.QtGui import QPainter, QPen, QBrush, QPixmap, QPolygonF, QTransform, QColor
from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer
import math

'''
객체 생성하면서 맵을 생성, 이후 위치, 방향을 받을 때마다 맵에 업데이트. 
'''
class MapViewer(QGraphicsView):
    def __init__(self, map_path, px_per_m_x, px_per_m_y):
        super().__init__()
        self.px_per_m_x = px_per_m_x 
        self.px_per_m_y = px_per_m_y
        # 실제세계 -> 픽셀 변환 비율. 미터당 픽셀.
        self.scene = QGraphicsScene() # 씬 객체 생성
        self.setScene(self.scene) # 앞으로 이 씬에서 작업을 하겠다.

        self.path_items = [] # 경로를 그리는 그래픽 아이템들을 저장할 리스트

        # 맵 이미지 로딩
        pixmap = QPixmap(map_path) 
        if pixmap.isNull():
            print(f"맵 이미지 로드 실패: {map_path}")
        else: # 맵 이미지 로드 및 초기화 작업들
            self.pixmap_item = QGraphicsPixmapItem(pixmap) #map을 아이템화
            self.pixmap_item.setTransformOriginPoint(self.pixmap_item.boundingRect().center()) # 이미지의 모서리 사각형을 따고, 그 중심을 잡아서 회전의 중심으로 함. 
            self.scene.addItem(self.pixmap_item) #씬에 지도 아이템을 추가한다.
            self.scene.setSceneRect(QRectF(pixmap.rect())) #지도의 이미지 크기를 사각형으로 잡고, 씬의 영역을 그 크기로 설정함. 이 영역 안에서만 아이템이 보이고 ,스크롤 바도 이를 기준으로 잡힘. 논리적 크기 설정.
            self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff) #스크롤 없앤다.
            self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff) #스크롤 없앤다.
            self.showFullScreen() #전체화면 
            #self.scale(2.0, 2.0) #확대

        # 디버그 텍스트
        self.debug_text = QGraphicsTextItem()
        self.debug_text.setDefaultTextColor(Qt.blue)
        self.debug_text.setPos(10, 10)
        
        self.scene.addItem(self.debug_text)

        self.est_marker = None
        self.heading_arrow = None

    def _init_est_items(self, x, y, heading):
        # 최초 한 번만 실행 → marker·arrow 생성
        # 1380, 1000
        px = x * self.px_per_m_x
        py = y * self.px_per_m_y
        radius = 5

        self.est_marker = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2)
        self.est_marker.setBrush(Qt.red)
        self.est_marker.setPen(QPen(Qt.red,1))
        self.est_marker.setZValue(20)
        self.est_marker.setPos(px, py)
        self.scene.addItem(self.est_marker)

        self.heading_arrow = QGraphicsPolygonItem()
        self.heading_arrow.setBrush(Qt.red)
        self.heading_arrow.setPen(QPen(Qt.red,1))
        self.heading_arrow.setZValue(19)
        self.scene.addItem(self.heading_arrow)
        self._update_arrow(px, py, heading)

        # 현재 상태 저장
        self._cur_pos = QPointF(px, py)
        self._cur_heading = heading
        self.centerOn(self._cur_pos)

    def _update_arrow(self, px, py, heading):
        rad = math.radians(heading)
        size = 13
        tip = QPointF(px + size * math.cos(rad), py + size * math.sin(rad))
        left = QPointF(px + size*0.5*math.cos(rad+math.radians(135)),
                       py + size*0.5*math.sin(rad+math.radians(135)))
        right = QPointF(px + size*0.5*math.cos(rad-math.radians(135)),
                        py + size*0.5*math.sin(rad-math.radians(135)))
        poly = QPolygonF([tip, left, right])
        self.heading_arrow.setPolygon(poly)

    def move_to(self, x, y, heading, duration=200):
        """
        x,y,heading → 부드럽게(duration ms) 보간
        """
        if self.est_marker is None or self.heading_arrow is None:
            self._init_est_items(x,y,heading)
            return

        start_pos = self._cur_pos
        end_pos = QPointF(x*self.px_per_m_x, y*self.px_per_m_y)
        start_h = self._cur_heading % 360
        end_h = heading % 360

        delta_h = (end_h - start_h + 180) % 360 - 180

        steps = 10
        for i in range(1, steps+1):
            t = i/steps
            px = start_pos.x() + (end_pos.x()-start_pos.x())*t
            py = start_pos.y() + (end_pos.y()-start_pos.y())*t
            hd = start_h + delta_h*t
            QTimer.singleShot(int(duration*t), 
                lambda px=px, py=py, hd=hd: (
                    self.est_marker.setPos(px, py),
                    self._update_arrow(px, py, hd),
                    self.centerOn(QPointF(px, py))
                )
            )

        # 최종 상태 업데이트
        self._cur_pos = end_pos
        print(self._cur_pos)
        self._cur_heading = (start_h + delta_h) % 360

        def draw_path(self, path_points):

            # 1. 기존에 그려진 경로가 있다면 삭제
            for item in self.path_items:
                self.scene.removeItem(item)
            self.path_items.clear()

            if not path_points or len(path_points) < 2:
                return

            # 2. 새로운 경로를 그릴 펜 설정 (빨간색, 두께 3)
            pen = QPen(QColor(255, 0, 0, 200), 3) # (R, G, B, 투명도)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)

            # 3. 경로의 각 지점을 선으로 연결
            for i in range(len(path_points) - 1):
                p1 = path_points[i]
                p2 = path_points[i+1]
                line = QGraphicsLineItem(p1.x(), p1.y(), p2.x(), p2.y())
                line.setPen(pen)
                line.setZValue(10)  # 마커(20)보다는 아래, 지도(0)보다는 위에 표시
                self.scene.addItem(line)
                self.path_items.append(line) # 삭제를 위해 리스트에 추가
