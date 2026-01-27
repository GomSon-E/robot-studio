from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from PySide6.QtCore import Qt, Signal


class DataCollectionPanel(QWidget):
    """데이터 수집 패널"""

    record_clicked = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # 헤더
        title = QLabel('Data Collection')
        title.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-size: 18px;
                font-weight: 600;
            }
        """)
        layout.addWidget(title)

        layout.addStretch()

        # Record 버튼
        self.record_btn = QPushButton('Record')
        self.record_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.record_btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: #ffffff;
                border: none;
                border-radius: 4px;
                padding: 12px 32px;
                font-size: 16px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #e53935;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)
        self.record_btn.clicked.connect(self._on_record_clicked)
        layout.addWidget(self.record_btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addStretch()

    def _on_record_clicked(self):
        """Record 버튼 클릭 시"""
        self.record_clicked.emit()
