from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsPixmapItem, QGraphicsPolygonItem
from PyQt5.QtGui import QPainter, QPen, QBrush, QPixmap, QPolygonF
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
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # 맵 이미지 로딩
        pixmap = QPixmap(map_path)
        if pixmap.isNull():
            print(f"맵 이미지 로드 실패: {map_path}")
        else:
            pixmap_item = QGraphicsPixmapItem(pixmap) #map을 아이템화
            self.scene.addItem(pixmap_item) #씬에 아이템 추가
            self.scene.setSceneRect(QRectF(pixmap.rect())) #씬의 크기를 맵 이미지 크기로 설정
            self.setMinimumSize(pixmap.width(), pixmap.height())#창의 최소 크기를 이미지 크기만큼 보장
            self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)#비율 유지하게, 꽉차도록 조정.

        # 디버그 텍스트
        self.debug_text = QGraphicsTextItem()
        self.debug_text.setDefaultTextColor(Qt.blue)
        self.debug_text.setPos(10, 10)
        self.scene.addItem(self.debug_text)

        self.est_marker = None
        self.heading_arrow = None

    def update_debug(self, rssi_vec, pos, dist_list):
        text = f"RSSI: {rssi_vec}\nPos: {pos.round(2)}\nDists: {dist_list.round(2)}"
        self.debug_text.setPlainText(text)

    def mark_estimated_position(self, x, y, heading_deg=0.0): #x,y,방향을 받아서 맵에 표시
        
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
        rad = math.radians(heading_deg)
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
