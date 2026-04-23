from pathlib import Path
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QPixmap

from .theme import (
    SIDEBAR_BG, SIDEBAR_BORDER,
    ACCENT, MENU_ICONS, render_svg_icon,
)

_LOGO_PATH = Path(__file__).parent.parent / 'assets' / 'logo.png'


def _load_logo(size: int = 28) -> QPixmap | None:
    px = QPixmap(str(_LOGO_PATH))
    if not px.isNull():
        return px.scaled(size, size,
                         Qt.AspectRatioMode.KeepAspectRatio,
                         Qt.TransformationMode.SmoothTransformation)
    return None


class SidebarItem(QWidget):
    """배지 아이콘 + 텍스트 메뉴 아이템"""

    clicked = Signal()

    def __init__(self, item_id: str, text: str, is_exit: bool = False, parent=None):
        super().__init__(parent)
        self.item_id = item_id
        self._is_exit = is_exit
        self._active = False
        self.setCursor(Qt.CursorShape.PointingHandCursor)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 9, 10, 9)
        layout.setSpacing(10)

        # 배지 컨테이너
        self._badge = QLabel()
        self._badge.setFixedSize(26, 26)
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._badge)

        # 텍스트
        self._label = QLabel(text)
        self._label.setStyleSheet("background: transparent; border: none;")
        layout.addWidget(self._label)
        layout.addStretch()

        self._update_style()

    def _update_style(self):
        if self._is_exit:
            badge_bg   = 'rgba(220, 38, 38, 0.18)'
            icon_color = 'rgba(220,38,38,0.85)'
            text_color = 'rgba(220, 38, 38, 0.9)'
            item_bg    = 'rgba(220, 38, 38, 0.10)'
        elif self._active:
            badge_bg   = ACCENT
            icon_color = 'white'
            text_color = '#4c1d95'
            item_bg    = 'rgba(255, 255, 255, 0.42)'
        else:
            badge_bg   = 'rgba(255, 255, 255, 0.18)'
            icon_color = 'rgba(255, 255, 255, 0.75)'
            text_color = 'rgba(255, 255, 255, 0.82)'
            item_bg    = 'transparent'

        # 배지 배경
        self._badge.setStyleSheet(f"""
            QLabel {{
                background-color: {badge_bg};
                border-radius: 8px;
            }}
        """)

        # 아이콘 렌더링
        svg_template = MENU_ICONS.get('exit' if self._is_exit else self.item_id)
        if svg_template:
            px = render_svg_icon(svg_template, icon_color, size=13)
            self._badge.setPixmap(px)

        # 텍스트 스타일
        weight = '600' if self._active else '400'
        self._label.setStyleSheet(
            f"color: {text_color}; font-size: 13px; font-weight: {weight};"
            " background: transparent; border: none;"
        )

        # 아이템 배경
        border = "border: 1px solid rgba(255,255,255,0.65);" if self._active else "border: none;"
        self.setStyleSheet(f"""
            QWidget {{
                background-color: {item_bg};
                border-radius: 10px;
                {border}
            }}
        """)

    def setChecked(self, checked: bool):
        if self._active != checked:
            self._active = checked
            self._update_style()

    def isChecked(self) -> bool:
        return self._active

    def mousePressEvent(self, event):
        self.clicked.emit()
        super().mousePressEvent(event)


class Sidebar(QWidget):
    """고정 너비 사이드바"""

    menu_selected = Signal(str)
    exit_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(210)
        self.setObjectName('Sidebar')
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(f"""
            #Sidebar {{
                background-color: {SIDEBAR_BG};
                border-right: 1px solid {SIDEBAR_BORDER};
            }}
            QWidget {{
                background-color: transparent;
                border: none;
            }}
        """)

        self._items: dict[str, SidebarItem] = {}
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 20, 12, 16)
        layout.setSpacing(4)

        # ── 브랜딩 ──────────────────────────────────────────────────────────
        brand_row = QHBoxLayout()
        brand_row.setContentsMargins(4, 0, 4, 0)
        brand_row.setSpacing(9)

        logo_pixmap = _load_logo(28)
        if logo_pixmap:
            logo_lbl = QLabel()
            logo_lbl.setPixmap(logo_pixmap)
            logo_lbl.setFixedSize(32, 32)
            logo_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            logo_lbl.setStyleSheet("""
                QLabel {
                    background-color: white;
                    border-radius: 9px;
                }
            """)
            brand_row.addWidget(logo_lbl)

        brand_text = QLabel('Robot Studio')
        brand_text.setStyleSheet("""
            QLabel {
                color: rgba(255, 255, 255, 0.95);
                font-size: 14px;
                font-weight: 700;
                letter-spacing: -0.3px;
            }
        """)
        brand_row.addWidget(brand_text)
        brand_row.addStretch()
        layout.addLayout(brand_row)

        layout.addSpacing(16)

        # ── 메뉴 레이블 ─────────────────────────────────────────────────────
        menu_lbl = QLabel('MENU')
        menu_lbl.setStyleSheet("""
            QLabel {
                color: rgba(255, 255, 255, 0.4);
                font-size: 9px; font-weight: 700;
                letter-spacing: 1.5px;
            }
        """)
        menu_lbl.setContentsMargins(10, 0, 0, 0)
        layout.addWidget(menu_lbl)

        layout.addSpacing(2)

        # ── 메뉴 아이템 ─────────────────────────────────────────────────────
        menus = [
            ('calibration',     'Calibration'),
            ('teleop',          'Teleop'),
            ('camera_preview',  'Camera Preview'),
            ('dataset_setting', 'Dataset Setting'),
        ]
        for item_id, label in menus:
            item = SidebarItem(item_id, label)
            item.clicked.connect(lambda _id=item_id: self._on_item_clicked(_id))
            layout.addWidget(item)
            self._items[item_id] = item

        layout.addStretch()

        # ── 구분선 ──────────────────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1)
        sep.setStyleSheet("background-color: rgba(255,255,255,0.2); border: none;")
        layout.addWidget(sep)

        layout.addSpacing(4)

        # ── Exit ────────────────────────────────────────────────────────────
        exit_item = SidebarItem('exit', 'Exit', is_exit=True)
        exit_item.clicked.connect(self.exit_requested.emit)
        layout.addWidget(exit_item)

    def _on_item_clicked(self, item_id: str):
        for _id, item in self._items.items():
            item.setChecked(_id == item_id)
        self.menu_selected.emit(item_id)
