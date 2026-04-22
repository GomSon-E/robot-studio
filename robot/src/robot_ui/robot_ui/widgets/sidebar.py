from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QFrame
)
from PySide6.QtCore import Signal, Qt

from .theme import (
    BG_SIDEBAR, BORDER, TEXT_BODY, TEXT_MUTED, TEXT_DISABLED,
    ACCENT, ACCENT_BG, ACCENT_ACTIVE_BG, ACCENT_RED,
)


class SidebarItem(QPushButton):
    """사이드바 메뉴 아이템"""

    def __init__(self, item_id: str, text: str, parent=None):
        super().__init__(text, parent)
        self.item_id = item_id
        self.setCheckable(True)
        self.setFixedHeight(40)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

        self.setStyleSheet(f"""
            QPushButton {{
                background-color: transparent;
                border: none;
                border-radius: 8px;
                color: {TEXT_BODY};
                font-size: 14px;
                text-align: left;
                padding-left: 16px;
            }}
            QPushButton:hover {{
                background-color: {ACCENT_BG};
            }}
            QPushButton:checked {{
                background-color: {ACCENT_ACTIVE_BG};
                color: {ACCENT};
                font-weight: 600;
            }}
        """)


class Sidebar(QWidget):
    """고정 너비 사이드바"""

    menu_selected = Signal(str)
    exit_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(220)
        self.setObjectName('Sidebar')
        self.setStyleSheet(f"""
            #Sidebar {{
                background-color: {BG_SIDEBAR};
                border-right: 1px solid {BORDER};
            }}
            QWidget {{
                background-color: {BG_SIDEBAR};
                border: none;
            }}
        """)

        self._items = {}
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 0, 12, 12)
        layout.setSpacing(4)

        # 앱 브랜딩
        brand = QLabel('Robot Studio')
        brand.setStyleSheet(f"""
            QLabel {{
                color: {ACCENT};
                font-size: 15px;
                font-weight: 700;
                letter-spacing: -0.3px;
                padding: 22px 4px 18px 4px;
                background-color: transparent;
                border: none;
            }}
        """)
        layout.addWidget(brand)

        sep_top = QFrame()
        sep_top.setFrameShape(QFrame.Shape.HLine)
        sep_top.setStyleSheet(f"background-color: {BORDER}; border: none; max-height: 1px;")
        layout.addWidget(sep_top)

        header = QLabel('MENU')
        header.setStyleSheet(f"""
            QLabel {{
                color: {TEXT_DISABLED};
                font-size: 10px;
                font-weight: 700;
                letter-spacing: 1.5px;
                padding: 12px 4px 8px 4px;
                background-color: transparent;
                border: none;
            }}
        """)
        layout.addWidget(header)

        calibration_item = SidebarItem('calibration', 'Calibration')
        calibration_item.clicked.connect(lambda: self._on_item_clicked('calibration'))
        layout.addWidget(calibration_item)
        self._items['calibration'] = calibration_item

        teleop_item = SidebarItem('teleop', 'Teleop')
        teleop_item.clicked.connect(lambda: self._on_item_clicked('teleop'))
        layout.addWidget(teleop_item)
        self._items['teleop'] = teleop_item

        camera_item = SidebarItem('camera_preview', 'Camera Preview')
        camera_item.clicked.connect(lambda: self._on_item_clicked('camera_preview'))
        layout.addWidget(camera_item)
        self._items['camera_preview'] = camera_item

        dataset_item = SidebarItem('dataset_setting', 'Dataset Setting')
        dataset_item.clicked.connect(lambda: self._on_item_clicked('dataset_setting'))
        layout.addWidget(dataset_item)
        self._items['dataset_setting'] = dataset_item

        layout.addStretch()

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"background-color: {BORDER}; border: none; max-height: 1px;")
        layout.addWidget(sep)

        exit_btn = QPushButton('Exit')
        exit_btn.setFixedHeight(40)
        exit_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        exit_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: transparent;
                border: none;
                border-radius: 8px;
                color: {ACCENT_RED};
                font-size: 14px;
                text-align: left;
                padding-left: 16px;
            }}
            QPushButton:hover {{
                background-color: #fef2f2;
            }}
        """)
        exit_btn.clicked.connect(self.exit_requested.emit)
        layout.addWidget(exit_btn)

    def _on_item_clicked(self, item_id: str):
        for id_, item in self._items.items():
            if id_ != item_id:
                item.setChecked(False)

        self.menu_selected.emit(item_id)
