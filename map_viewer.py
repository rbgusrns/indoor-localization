import sys
import math
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsPolygonItem
from PyQt5.QtGui import QPainter, QPen, QBrush, QPixmap, QPolygonF, QColor, QPainterPath
from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer, QVariantAnimation, QEasingCurve, QAbstractAnimation

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

        # 사용자 관련 변수
        self.est_marker = None
        self.heading_arrow = None
        self._cur_heading = 0.0
        
        # 로봇 관련 변수
        self.robot_marker = None

    def _start_animation(self, target_item, end_pos, duration=300):
        """QVariantAnimation을 사용해 target_item을 부드럽게 이동시키는 공통 함수"""
        animation = QVariantAnimation(self)
        animation.setStartValue(target_item.pos())
        animation.setEndValue(end_pos)
        animation.setDuration(duration)
        animation.setEasingCurve(QEasingCurve.InOutCubic)

        # 애니메이션 값이 변할 때마다 target_item의 위치(setPos)를 업데이트하도록 연결
        animation.valueChanged.connect(target_item.setPos)

        # 애니메이션이 끝나면 자동으로 소멸되도록 설정하여 메모리 누수 방지
        animation.start(QAbstractAnimation.DeleteWhenStopped)

    def _init_est_items(self, x, y, heading):
        """사용자 위치 마커와 방향 화살표를 초기화합니다."""
        px = x * self.px_per_m_x
        py = y * self.px_per_m_y
        radius = 8

        self.est_marker = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2)
        self.est_marker.setBrush(QColor("#e74c3c")) # 빨간색
        self.est_marker.setPen(QPen(Qt.NoPen))
        self.est_marker.setZValue(20)
        self.scene.addItem(self.est_marker)

        self.heading_arrow = QGraphicsPolygonItem()
        self.heading_arrow.setBrush(QColor("#e74c3c"))
        self.heading_arrow.setPen(QPen(Qt.NoPen))
        self.heading_arrow.setZValue(19)
        self.scene.addItem(self.heading_arrow)
        
        self.est_marker.setPos(px, py) # 초기 위치 설정
        self._update_arrow_and_center(px, py, heading)

    def _update_arrow_and_center(self, px, py, heading):
        """방향 화살표를 업데이트하고, 화면을 해당 위치로 중앙 정렬합니다."""
        self._cur_heading = heading
        
        # 화살표 모양 정의 (원점 기준)
        size = 15
        arrow_poly = QPolygonF([
            QPointF(size, 0),
            QPointF(size*0.2, -size*0.4),
            QPointF(size*0.2, size*0.4)
        ])
        self.heading_arrow.setPolygon(arrow_poly)

        # 화살표 위치 및 회전 설정
        self.heading_arrow.setPos(px, py)
        self.heading_arrow.setRotation(heading)

        self.centerOn(QPointF(px, py))

    def move_to(self, x, y, heading, duration=300):
        """사용자 아이콘을 부드럽게 이동시킵니다."""
        if self.est_marker is None:
            self._init_est_items(x, y, heading)
            return

        end_pos = QPointF(x * self.px_per_m_x, y * self.px_per_m_y)
        
        # 사용자 마커와 화살표 위치를 동시에 애니메이션으로 이동
        self._start_animation(self.est_marker, end_pos, duration)
        self._start_animation(self.heading_arrow, end_pos, duration)
        
        # 화살표 방향은 즉시 업데이트
        self.heading_arrow.setRotation(heading)
        self._cur_heading = heading
        
        # 화면 중앙 정렬은 애니메이션 없이 즉시
        self.centerOn(end_pos)

    def mark_estimated_position(self, x, y, heading):
        """사용자 아이콘을 즉시 이동시킵니다 (애니메이션 없음)."""
        if self.est_marker is None:
            self._init_est_items(x, y, heading)
            return
        
        px = x * self.px_per_m_x
        py = y * self.px_per_m_y
        
        self.est_marker.setPos(px, py)
        self._update_arrow_and_center(px, py, heading)

    def draw_path(self, path_points):
        """주어진 좌표들을 따라 지도 위에 경로를 그립니다."""
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

    # --- ▼ 로봇 관련 메서드들 ▼ ---
    
    def _init_robot_marker(self, px, py):
        """로봇 마커를 처음 생성하고 초기화합니다."""
        radius = 10
        self.robot_marker = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2)
        self.robot_marker.setBrush(QColor("#9d26ff")) # 보라색
        self.robot_marker.setPen(QPen(QColor(Qt.white), 2))
        self.robot_marker.setZValue(18)
        self.scene.addItem(self.robot_marker)
        self.robot_marker.setPos(px, py)

    def update_robot_position(self, px, py, duration=300):
        """지도 위에 로봇의 위치를 즉시 표시하거나 업데이트합니다 (순간이동)."""
        if self.robot_marker is None:
            # 마커가 없으면 새로 생성하고 위치를 설정합니다.
            self._init_robot_marker(px, py)
        else:
            # 마커가 이미 있으면 즉시 해당 위치로 이동시킵니다.
            self.robot_marker.setPos(px, py)