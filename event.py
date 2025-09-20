import sys
from PyQt5.QtWidgets import QApplication, QDialog, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt

class SelectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_room = None
        self.setWindowTitle("목적지 선택")
        
        # 1. 다이얼로그 자체에 ID 부여
        self.setObjectName("SelectionDialog")

        layout = QVBoxLayout(self)
        layout.setSpacing(15) # 버튼 사이 간격
        layout.setAlignment(Qt.AlignCenter)

        title_label = QLabel("진료실을 선택해 주세요.")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # 예시 목적지 리스트 (실제로는 config에서 가져오는 것이 좋습니다)
        rooms = ["101호", "102호", "103호"]
        for room in rooms:
            btn = QPushButton(room)
            
            # 2. 각 버튼에 스타일링을 위한 '클래스' 속성 부여
            btn.setProperty("class", "dialog_button")
            
            btn.clicked.connect(self.on_room_selected)
            layout.addWidget(btn)

    def on_room_selected(self):
        self.selected_room = self.sender().text()
        self.accept()

    def exec_(self):
        # 다이얼로그를 부모 위젯 중앙에 위치시키기
        if self.parent():
            parent_rect = self.parent().geometry()
            self.move(parent_rect.center() - self.rect().center())
        return super().exec_()

# 테스트를 위한 코드
if __name__ == '__main__':
    app = QApplication(sys.argv)
    # 스타일시트 로드 (테스트 시 확인을 위해)
    # app.setStyleSheet(open('stylesheet.qss').read())
    dialog = SelectionDialog()
    if dialog.exec_():
        print(f"선택된 목적지: {dialog.selected_room}")
    sys.exit()