import sys
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsPolygonItem
from PyQt5.QtGui import QPainter, QPen, QBrush, QPixmap, QPolygonF, QColor, QPainterPath
from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer
import math

class MapViewer(QGraphicsView):
    """
    지도, 사용자 위치, 경로 등 모든 시각적 요소를 관리하는 클래스.
    """
    def __init__(self, map_path, px_per_m_x, px_per_m_y):
        super().__init__()
        self.px_per_m_x = px_per_m_x 
        self.px_per_m_y = px_per_m_y
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        self.path_item = None

        pixmap = QPixmap(map_path) 
        if pixmap.isNull():
            print(f"맵 이미지 로드 실패: {map_path}")
        else:
            self.pixmap_item = QGraphicsPixmapItem(pixmap)
            self.scene.addItem(self.pixmap_item)
            self.scene.setSceneRect(QRectF(pixmap.rect()))
            self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.showFullScreen()

        self.est_marker = None
        self.heading_arrow = None
        
        # 로봇 위치 마커를 위한 변수
        self.robot_marker = None

    def _init_est_items(self, x, y, heading):
        """사용자 위치 마커와 방향 화살표를 초기화합니다."""
        px = x * self.px_per_m_x
        py = y * self.px_per_m_y
        radius = 8

        self.est_marker = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2)
        self.est_marker.setBrush(QColor("#e74c3c")) # 빨간색
        self.est_marker.setPen(QPen(Qt.NoPen))
        self.est_marker.setZValue(20)
        self.est_marker.setPos(px, py)
        self.scene.addItem(self.est_marker)

        self.heading_arrow = QGraphicsPolygonItem()
        self.heading_arrow.setBrush(QColor("#e74c3c"))
        self.heading_arrow.setPen(QPen(Qt.NoPen))
        self.heading_arrow.setZValue(19)
        self.scene.addItem(self.heading_arrow)
        self._update_arrow(px, py, heading)

        self._cur_pos = QPointF(px, py)
        self._cur_heading = heading
        self.centerOn(self._cur_pos)

    def _update_arrow(self, px, py, heading):
        """주어진 좌표와 방향으로 화살표 폴리곤을 업데이트합니다."""
        rad = math.radians(heading)
        size = 15
        tip = QPointF(px + size * math.cos(rad), py + size * math.sin(rad))
        left = QPointF(px + size*0.6*math.cos(rad+math.radians(135)),
                       py + size*0.6*math.sin(rad+math.radians(135)))
        right = QPointF(px + size*0.6*math.cos(rad-math.radians(135)),
                        py + size*0.6*math.sin(rad-math.radians(135)))
        poly = QPolygonF([tip, left, right])
        self.heading_arrow.setPolygon(poly)

    def move_to(self, x, y, heading, duration=200):
        """사용자 아이콘을 부드럽게 이동시킵니다."""
        if self.est_marker is None or self.heading_arrow is None:
            self._init_est_items(x, y, heading)
            return

        start_pos, end_pos = self._cur_pos, QPointF(x * self.px_per_m_x, y * self.px_per_m_y)
        start_h, end_h = self._cur_heading % 360, heading % 360
        delta_h = (end_h - start_h + 180) % 360 - 180
        
        steps = 10
        for i in range(1, steps + 1):
            t = i / steps
            px = start_pos.x() + (end_pos.x() - start_pos.x()) * t
            py = start_pos.y() + (end_pos.y() - start_pos.y()) * t
            hd = start_h + delta_h * t
            QTimer.singleShot(int(duration * t), lambda px=px, py=py, hd=hd: self._update_marker_pos(px, py, hd))

        self._cur_pos, self._cur_heading = end_pos, (start_h + delta_h) % 360

    def _update_marker_pos(self, px, py, hd):
        """마커와 화살표의 위치를 업데이트하는 내부 함수."""
        self.est_marker.setPos(px, py)
        self._update_arrow(px, py, hd)
        self.centerOn(QPointF(px, py))

    def mark_estimated_position(self, x, y, heading):
        """사용자 아이콘을 즉시 이동시킵니다 (애니메이션 없음)."""
        if self.est_marker is None or self.heading_arrow is None:
            self._init_est_items(x, y, heading)
            return
        
        px = x * self.px_per_m_x
        py = y * self.px_per_m_y
        self._update_marker_pos(px, py, heading)
        self._cur_pos, self._cur_heading = QPointF(px, py), heading

    def draw_path(self, path_points):
        """
        주어진 좌표들을 따라 지도 위에 경로를 그립니다. (화살표 없음)
        """
        if self.path_item:
            self.scene.removeItem(self.path_item)
            self.path_item = None
        
        if not path_points or len(path_points) < 2:
            return
        
        path = QPainterPath()
        path.moveTo(path_points[0])
        for point in path_points[1:]:
            path.lineTo(point)
        
        pen = QPen(QColor("#3498db"), 8, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        self.path_item = self.scene.addPath(path, pen)
        self.path_item.setZValue(10)

    def update_robot_position(self, px, py):
        """지도 위에 로봇의 위치를 표시하거나 업데이트합니다."""
        if self.robot_marker is None:
            # 로봇 마커가 없으면 새로 생성
            radius = 10
            self.robot_marker = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2)
            # 사용자와 다른 색상(파란색)으로 설정
            self.robot_marker.setBrush(QColor("#3498db")) 
            self.robot_marker.setPen(QPen(QColor(Qt.white), 2))
            self.robot_marker.setZValue(18) # 사용자 마커보다 살짝 뒤에 위치
            self.scene.addItem(self.robot_marker)
        
        # 로봇 마커의 위치를 업데이트
        self.robot_marker.setPos(px, py)