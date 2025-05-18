from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsPixmapItem, QGraphicsPolygonItem
from PyQt5.QtGui import QPainter, QPen, QBrush, QPixmap, QPolygonF, QTransform
from PyQt5.QtCore import Qt, QRectF, QPointF
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

    def update_heading(self, heading_deg): #나는 중심에 있고, 지도가 회전하도록 하는 메서드
        """heading_deg: 실제 heading (내가 바라보는 각도)"""
        rotation_angle = -heading_deg  # 지도는 반대 방향으로 회전
        transform = QTransform().rotate(rotation_angle)
        self.pixmap_item.setTransform(transform)
            
    def update_debug(self, rssi_vec, pos, dist_list):
        text = f"RSSI: {rssi_vec}\nPos: {pos.round(2)}\nDists: {dist_list.round(2)}"
        self.debug_text.setPlainText(text)



    def mark_estimated_position(self, x, y,heading): #x,y,방향을 받아서 맵에 표시
        
        if self.est_marker:
            self.scene.removeItem(self.est_marker)
        if self.heading_arrow:
            self.scene.removeItem(self.heading_arrow)

        # 실제 좌표 → 픽셀
        px = x * self.px_per_m_x
        py = y * self.px_per_m_y

        # 위치 마커
        radius = 5
        marker = QGraphicsEllipseItem(px - radius, py - radius, radius * 2, radius * 2) #위치 및 크기 설정
        marker.setBrush(Qt.red)
        marker.setPen(QPen(Qt.red, 1))
        marker.setZValue(20)
        self.scene.addItem(marker)
        self.est_marker = marker

        # 삼각형 화살표
        rad = math.radians(heading)
        size = 13  # 삼각형 크기

        # 삼각형 정점 계산
        tip = QPointF(px + size * math.cos(rad), py + size * math.sin(rad))
        left = QPointF(px + size * 0.5 * math.cos(rad + math.radians(135)),
                       py + size * 0.5 * math.sin(rad + math.radians(135)))
        right = QPointF(px + size * 0.5 * math.cos(rad - math.radians(135)),
                        py + size * 0.5 * math.sin(rad - math.radians(135)))

        triangle = QPolygonF([tip, left, right])
        arrow = QGraphicsPolygonItem(triangle)
        arrow.setBrush(QBrush(Qt.red))
        arrow.setPen(QPen(Qt.red, 1))
        arrow.setZValue(19)
        self.scene.addItem(arrow)
        self.heading_arrow = arrow
        

