import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QDialog, QLabel

# 2. 선택을 위한 새로운 다이얼로그(QDialog) 창 정의
class SelectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("진료실 선택")
        self.setFixedSize(300, 200)

        # 다이얼로그 창에 들어갈 레이아웃과 위젯들
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.selected_room = None  # 선택된 진료실 이름을 저장할 변수

        # 진료실 버튼 목록
        rooms = ["101호", "102호", "103호"]

        for room_name in rooms:
            button = QPushButton(room_name)
            # 버튼 클릭 시그널을 room_selected 슬롯에 연결
            # lambda를 사용하여 어떤 버튼이 눌렸는지 이름을 전달
            button.clicked.connect(lambda checked, name=room_name: self.room_selected(name))
            self.layout.addWidget(button)

    # 버튼이 클릭되었을 때 실행될 슬롯(메서드)
    def room_selected(self, name):
        self.selected_room = name
        self.accept() # QDialog를 닫고, '성공' 상태(Accepted)를 반환

# 1. 메인 윈도우 정의
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("메인 화면")
        self.setGeometry(100, 100, 400, 300)

        # 메인 위젯 및 레이아웃 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 결과를 표시할 라벨
        self.result_label = QLabel("선택된 진료실이 여기에 표시됩니다.")
        layout.addWidget(self.result_label)

        # 다이얼로그를 띄울 버튼
        self.select_button = QPushButton("진료실 선택하기")
        self.select_button.clicked.connect(self.show_selection_dialog) # 버튼 클릭 시그널 -> 슬롯 연결
        layout.addWidget(self.select_button)


    # '진료실 선택하기' 버튼이 눌렸을 때 실행될 슬롯(메서드)
    def show_selection_dialog(self):
        # SelectionDialog 인스턴스 생성
        dialog = SelectionDialog(self)

        # exec()는 다이얼로그가 닫힐 때까지 코드 실행을 멈춤
        # 사용자가 선택을 완료하면 result에 QDialog.Accepted(값: 1) 또는 QDialog.Rejected(값: 0)가 담김
        result = dialog.exec()

        # 만약 사용자가 진료실을 선택했다면(dialog.accept()가 호출되었다면)
        if result:
            selected = dialog.selected_room
            self.result_label.setText(f"'{selected}'을(를) 선택하셨습니다.")
            print(f"선택된 진료실: {selected}")
        else:
            self.result_label.setText("진료실 선택을 취소했습니다.")
            print("선택이 취소되었습니다.")


# 애플리케이션 실행
if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())