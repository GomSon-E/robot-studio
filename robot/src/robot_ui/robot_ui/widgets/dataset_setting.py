from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import Qt


class DatasetSettingPanel(QWidget):
    """데이터셋 설정 패널"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_topic: str = ""
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # 헤더
        title = QLabel('Dataset Setting')
        title.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-size: 18px;
                font-weight: 600;
            }
        """)
        layout.addWidget(title)

        # 선택된 카메라 표시
        self.camera_label = QLabel('No camera selected')
        self.camera_label.setStyleSheet("""
            QLabel {
                color: #858585;
                font-size: 14px;
            }
        """)
        layout.addWidget(self.camera_label)

        layout.addStretch()

    def set_camera(self, topic_name: str):
        """선택된 카메라 설정"""
        self.selected_topic = topic_name
        self.camera_label.setText(f'Selected: {topic_name}')
