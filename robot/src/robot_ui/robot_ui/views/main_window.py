from PySide6.QtWidgets import QMainWindow, QWidget, QHBoxLayout
from ..widgets import Sidebar


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Robot Studio')
        self.setMinimumSize(1200, 800)

        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QWidget {
                background-color: #1e1e1e;
                color: #cccccc;
            }
        """)

        self._setup_ui()

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 1. 고정 사이드바
        self.sidebar = Sidebar()
        main_layout.addWidget(self.sidebar)

        # 2. 메인 콘텐츠 영역 (빈 영역)
        self.main_area = QWidget()
        self.main_area.setStyleSheet("background-color: #1e1e1e;")
        main_layout.addWidget(self.main_area, 1)
