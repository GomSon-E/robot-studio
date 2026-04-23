from PySide6.QtGui import QPainter, QPixmap
from PySide6.QtCore import Qt

FONT_FAMILY = "'Pretendard', '-apple-system', 'BlinkMacSystemFont', 'Segoe UI', 'Noto Sans KR', sans-serif"

# ── 텍스트 ────────────────────────────────────────────────────────────────────
TEXT_H1       = '#1a1d2e'
TEXT_BODY     = '#52586b'
TEXT_MUTED    = '#8e95a8'
TEXT_DISABLED = '#adb5c4'

# ── 포인트 컬러 ───────────────────────────────────────────────────────────────
ACCENT           = '#7c3aed'
ACCENT_HOVER     = '#6d28d9'
ACCENT_GREEN     = '#059669'
ACCENT_GREEN_END = '#34d399'
ACCENT_RED       = '#dc2626'
ACCENT_RED_END   = '#ef4444'

# ── 배경 (앱 루트 QSS 용) ─────────────────────────────────────────────────────
BG_GRADIENT = (
    "qlineargradient(x1:0, y1:0, x2:1, y2:1,"
    " stop:0 #ede9fe, stop:0.35 #ddd6fe,"
    " stop:0.70 #c7d2fe, stop:1.0 #bfdbfe)"
)

# ── 글래스 카드 ───────────────────────────────────────────────────────────────
GLASS_BG     = 'rgba(255, 255, 255, 133)'   # 52%
GLASS_BORDER = 'rgba(255, 255, 255, 184)'   # 72%
GLASS_BG_LT  = 'rgba(255, 255, 255, 89)'    # 35%
GLASS_BOR_LT = 'rgba(255, 255, 255, 115)'   # 45%

# ── 사이드바 ──────────────────────────────────────────────────────────────────
SIDEBAR_BG     = 'rgba(255, 255, 255, 56)'   # 22%
SIDEBAR_BORDER = 'rgba(255, 255, 255, 115)'  # 45%

# ── 기타 ──────────────────────────────────────────────────────────────────────
BORDER       = '#e2e5ec'
BORDER_FOCUS = '#7c3aed'

RADIUS_LG  = '14px'
RADIUS_MD  = '10px'
RADIUS_SM  = '7px'
RADIUS_BAR = '4px'

# ── SVG 아이콘 문자열 (stroke 색상은 {color} placeholder) ───────────────────
_SVG_CALIBRATION = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="13" height="13"'
    ' viewBox="0 0 24 24" fill="none" stroke="{color}"'
    ' stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
    '<circle cx="12" cy="12" r="3"/>'
    '<path d="M12 5v2M12 17v2M5 12H3M21 12h-2'
    'M7.05 7.05 5.636 5.636M18.364 18.364l-1.414-1.414'
    'M7.05 16.95 5.636 18.364M18.364 5.636 16.95 7.05"/>'
    '</svg>'
)
_SVG_TELEOP = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="13" height="13"'
    ' viewBox="0 0 24 24" fill="none" stroke="{color}"'
    ' stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
    '<path d="M12 2L2 7l10 5 10-5-10-5z"/>'
    '<path d="M2 17l10 5 10-5"/>'
    '<path d="M2 12l10 5 10-5"/>'
    '</svg>'
)
_SVG_CAMERA = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="13" height="13"'
    ' viewBox="0 0 24 24" fill="none" stroke="{color}"'
    ' stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
    '<path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8'
    ' a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/>'
    '<circle cx="12" cy="13" r="4"/>'
    '</svg>'
)
_SVG_DATASET = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="13" height="13"'
    ' viewBox="0 0 24 24" fill="none" stroke="{color}"'
    ' stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
    '<rect x="3" y="3" width="18" height="18" rx="2"/>'
    '<path d="M3 9h18M9 21V9"/>'
    '</svg>'
)
_SVG_EXIT = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="13" height="13"'
    ' viewBox="0 0 24 24" fill="none" stroke="{color}"'
    ' stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
    '<path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4"/>'
    '<polyline points="16 17 21 12 16 7"/>'
    '<line x1="21" y1="12" x2="9" y2="12"/>'
    '</svg>'
)

MENU_ICONS = {
    'calibration':    _SVG_CALIBRATION,
    'teleop':         _SVG_TELEOP,
    'camera_preview': _SVG_CAMERA,
    'dataset_setting': _SVG_DATASET,
    'exit':           _SVG_EXIT,
}


def render_svg_icon(svg_template: str, color: str, size: int = 13) -> QPixmap:
    """SVG 문자열을 QPixmap으로 렌더링한다. color는 stroke 색상 문자열."""
    try:
        from PySide6.QtSvg import QSvgRenderer
        from PySide6.QtCore import QByteArray
        raw = svg_template.format(color=color).encode('utf-8')
        renderer = QSvgRenderer(QByteArray(raw))
        px = QPixmap(size, size)
        px.fill(Qt.GlobalColor.transparent)
        p = QPainter(px)
        renderer.render(p)
        p.end()
        return px
    except Exception as exc:
        import logging
        logging.getLogger(__name__).warning("render_svg_icon failed: %s", exc)
        return QPixmap()


# ── QSS 헬퍼 ──────────────────────────────────────────────────────────────────

def glass_frame(radius: str = RADIUS_LG) -> str:
    return f"""
        QFrame {{
            background-color: {GLASS_BG};
            border: 1px solid {GLASS_BORDER};
            border-radius: {radius};
        }}
    """


def btn_primary(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 {ACCENT}, stop:1 #a78bfa);
            color: white; border: none; border-radius: {RADIUS_MD};
            padding: 10px 20px; font-weight: 600; font-size: 12px; {h}
        }}
        QPushButton:hover {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 {ACCENT_HOVER}, stop:1 {ACCENT});
        }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_success(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 {ACCENT_GREEN}, stop:1 {ACCENT_GREEN_END});
            color: white; border: none; border-radius: {RADIUS_MD};
            padding: 10px 20px; font-weight: 600; font-size: 12px; {h}
        }}
        QPushButton:hover {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #047857, stop:1 {ACCENT_GREEN});
        }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_danger(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 {ACCENT_RED}, stop:1 {ACCENT_RED_END});
            color: white; border: none; border-radius: {RADIUS_MD};
            padding: 10px 20px; font-weight: 700; font-size: 12px; {h}
        }}
        QPushButton:hover {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #b91c1c, stop:1 {ACCENT_RED});
        }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_warning(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #d97706, stop:1 #f59e0b);
            color: white; border: none; border-radius: {RADIUS_MD};
            padding: 10px 20px; font-weight: 600; font-size: 12px; {h}
        }}
        QPushButton:hover {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #b45309, stop:1 #d97706);
        }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_ghost() -> str:
    return f"""
        QPushButton {{
            background-color: {GLASS_BG}; color: {TEXT_BODY};
            border: 1px solid {GLASS_BORDER}; border-radius: {RADIUS_MD};
            padding: 10px 20px; font-size: 14px;
        }}
        QPushButton:hover {{ background-color: {GLASS_BG_LT}; }}
    """


def btn_back() -> str:
    return f"""
        QPushButton {{
            background-color: transparent; color: {TEXT_MUTED};
            border: none; padding: 8px 4px; font-size: 13px;
        }}
        QPushButton:hover {{ color: {TEXT_BODY}; }}
    """


def btn_icon_sm() -> str:
    return f"""
        QPushButton {{
            background-color: {GLASS_BG}; border: 1px solid {GLASS_BORDER};
            border-radius: {RADIUS_SM}; color: {TEXT_BODY}; font-size: 14px;
        }}
        QPushButton:hover {{ background-color: {GLASS_BG_LT}; }}
    """


def combobox_style() -> str:
    return f"""
        QComboBox {{
            background-color: {GLASS_BG};
            border: 1px solid {GLASS_BORDER};
            border-radius: {RADIUS_MD};
            padding: 6px 10px; color: {TEXT_H1}; font-size: 14px;
        }}
        QComboBox:focus {{ border-color: {BORDER_FOCUS}; }}
        QComboBox::drop-down {{ border: none; width: 20px; }}
        QComboBox QAbstractItemView {{
            background-color: white; color: {TEXT_BODY};
            selection-background-color: rgba(124,58,237,0.1);
            border: 1px solid {BORDER}; outline: none;
        }}
    """


def scrollbar_style() -> str:
    return f"""
        QScrollArea {{ border: none; background-color: transparent; }}
        QScrollBar:vertical {{
            background-color: rgba(196,181,253,0.2); width: 8px; border-radius: 4px;
        }}
        QScrollBar::handle:vertical {{
            background-color: rgba(124,58,237,0.3); border-radius: 4px; min-height: 20px;
        }}
        QScrollBar::handle:vertical:hover {{
            background-color: rgba(124,58,237,0.5);
        }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}
    """


def progressbar_style(start_color: str, end_color: str | None = None) -> str:
    end = end_color or start_color
    return f"""
        QProgressBar {{
            border: none; border-radius: {RADIUS_BAR};
            background-color: rgba(196, 181, 253, 64);
        }}
        QProgressBar::chunk {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 {start_color}, stop:1 {end});
            border-radius: {RADIUS_BAR};
        }}
    """


def groupbox_style() -> str:
    return f"""
        QGroupBox {{
            color: {TEXT_MUTED}; font-size: 11px; font-weight: 600;
            letter-spacing: 0.5px; text-transform: uppercase;
            background-color: {GLASS_BG};
            border: 1px solid {GLASS_BORDER}; border-radius: {RADIUS_LG};
            margin-top: 10px; padding-top: 12px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin; left: 12px; padding: 0 4px;
        }}
    """


def messagebox_style() -> str:
    return f"""
        QMessageBox {{ background-color: white; color: {TEXT_H1}; }}
        QLabel {{ color: {TEXT_BODY}; font-size: 14px; }}
        QPushButton {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 {ACCENT}, stop:1 #a78bfa);
            color: white; border: none; border-radius: {RADIUS_MD};
            padding: 8px 20px; font-weight: 600; min-width: 80px;
        }}
        QPushButton:hover {{
            background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 {ACCENT_HOVER}, stop:1 {ACCENT});
        }}
    """


# ── 하위 호환 별칭 (Tasks 2–7 완료 시 제거) ──────────────────────────────────
BG_PAGE          = BG_GRADIENT
BG_CARD          = GLASS_BG
BG_INPUT         = GLASS_BG
BG_SURFACE       = GLASS_BG_LT
BG_SIDEBAR       = SIDEBAR_BG
ACCENT_BG        = 'rgba(124, 58, 237, 20)'
ACCENT_ACTIVE_BG = 'rgba(124, 58, 237, 31)'
