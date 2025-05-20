from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsPixmapItem, QGraphicsPolygonItem
from PyQt5.QtGui import QPainter, QPen, QBrush, QPixmap, QPolygonF, QTransform
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

        # 맵 이미지 로딩
        pixmap = QPixmap(map_path) 
        if pixmap.isNull():
            print(f"맵 이미지 로드 실패: {map_path}")
        else: # 맵 이미지 로드 및 초기화 작업들
            self.pixmap_item = QGraphicsPixmapItem(pixmap) #map을 아이템화
            self.pixmap_item.setTransformOriginPoint(self.pixmap_item.boundingRect().center()) # 이미지의 모서리 사각형을 따고, 그 중심을 잡아서 회전의 중심으로 함. 
            self.scene.addItem(self.pixmap_item) #씬에 지도 아이템을 추가한다.
            self.scene.setSceneRect(QRectF(pixmap.rect())) #지도의 이미지 크기를 사각형으로 잡고, 씬의 영역을 그 크기로 설정함. 이 영역 안에서만 아이템이 보이고 ,스크롤 바도 이를 기준으로 잡힘. 논리적 크기 설정.
            #self.setMinimumSize(pixmap.width(), pixmap.height()) #이 창은 이미지 크기보다 작아질 수가 없음.
            self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)#비율 유지하게, 꽉차도록 조정...?
            self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.showFullScreen()
        # 디버그 텍스트
        self.debug_text = QGraphicsTextItem()
        self.debug_text.setDefaultTextColor(Qt.blue)
        self.debug_text.setPos(10, 10)
        
        self.scene.addItem(self.debug_text)

        self.est_marker = None
        self.heading_arrow = None

    def _init_est_items(self, x, y, heading):
        # 최초 한 번만 실행 → marker·arrow 생성
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
        start_h = self._cur_heading
        end_h = heading

        steps = 10
        for i in range(1, steps+1):
            t = i/steps
            px = start_pos.x() + (end_pos.x()-start_pos.x())*t
            py = start_pos.y() + (end_pos.y()-start_pos.y())*t
            hd = start_h + (end_h-start_h)*t
            QTimer.singleShot(int(duration*t), 
                lambda px=px, py=py, hd=hd: (
                    self.est_marker.setPos(px, py),
                    self._update_arrow(px, py, hd)
                )
            )

        # 최종 상태 업데이트
        self._cur_pos = end_pos
        self._cur_heading = end_h