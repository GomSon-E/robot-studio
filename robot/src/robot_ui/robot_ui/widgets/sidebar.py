from pathlib import Path
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QPixmap, QPainter

from .theme import (
    BG_SIDEBAR, BORDER, TEXT_BODY, TEXT_MUTED, TEXT_DISABLED,
    ACCENT, ACCENT_BG, ACCENT_ACTIVE_BG, ACCENT_RED,
)

_LOGO_PATH = Path(__file__).parent.parent / 'assets' / 'logo.svg'


def _load_logo(size: int = 28) -> QPixmap | None:
    try:
        from PySide6.QtSvg import QSvgRenderer
        renderer = QSvgRenderer(str(_LOGO_PATH))
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)
        p = QPainter(pixmap)
        renderer.render(p)
        p.end()
        return pixmap
    except Exception:
        pass
    # QPixmap direct SVG fallback
    px = QPixmap(str(_LOGO_PATH))
    if not px.isNull():
        return px.scaled(size, size, Qt.AspectRatioMode.KeepAspectRatio,
                         Qt.TransformationMode.SmoothTransformation)
    return None


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

        # ── 앱 브랜딩 ──────────────────────────────────────────
        brand_row = QHBoxLayout()
        brand_row.setContentsMargins(4, 20, 4, 18)
        brand_row.setSpacing(8)

        logo_pixmap = _load_logo(28)
        if logo_pixmap:
            logo_lbl = QLabel()
            logo_lbl.setPixmap(logo_pixmap)
            logo_lbl.setFixedSize(28, 28)
            logo_lbl.setStyleSheet("background: transparent; border: none;")
            brand_row.addWidget(logo_lbl)

        brand_text = QLabel('Robot Studio')
        brand_text.setStyleSheet(f"""
            QLabel {{
                color: {ACCENT};
                font-size: 15px;
                font-weight: 700;
                letter-spacing: -0.3px;
                background-color: transparent;
                border: none;
            }}
        """)
        brand_row.addWidget(brand_text)
        brand_row.addStretch()
        layout.addLayout(brand_row)

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
