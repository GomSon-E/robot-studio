# Design tokens aligned with the web UI's purple theme
FONT_FAMILY = "'Pretendard', '-apple-system', 'BlinkMacSystemFont', 'Segoe UI', 'Noto Sans KR', sans-serif"

BG_PAGE    = '#f4f5f8'
BG_CARD    = '#ffffff'
BG_INPUT   = '#ffffff'
BG_SURFACE = '#f9fafb'
BG_SIDEBAR = '#ffffff'

TEXT_H1       = '#1a1d2e'
TEXT_BODY     = '#52586b'
TEXT_MUTED    = '#8e95a8'
TEXT_DISABLED = '#adb5c4'

ACCENT             = '#7c3aed'
ACCENT_HOVER       = '#6d28d9'
ACCENT_BG          = '#f5f3ff'
ACCENT_ACTIVE_BG   = '#ede9fd'
ACCENT_BORDER      = '#ddd6fe'
ACCENT_GREEN       = '#059669'
ACCENT_GREEN_HOVER = '#047857'
ACCENT_RED         = '#dc2626'
ACCENT_RED_HOVER   = '#b91c1c'

BORDER       = '#e2e5ec'
BORDER_FOCUS = '#7c3aed'

RADIUS_LG = '12px'
RADIUS_MD  = '8px'
RADIUS_SM  = '4px'


def btn_primary(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background-color: {ACCENT}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 10px 20px;
            font-weight: 600; font-size: 14px; {h}
        }}
        QPushButton:hover {{ background-color: {ACCENT_HOVER}; }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_success(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background-color: {ACCENT_GREEN}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 10px 20px;
            font-weight: 600; font-size: 14px; {h}
        }}
        QPushButton:hover {{ background-color: {ACCENT_GREEN_HOVER}; }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_danger(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background-color: {ACCENT_RED}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 10px 20px;
            font-weight: 600; font-size: 14px; {h}
        }}
        QPushButton:hover {{ background-color: {ACCENT_RED_HOVER}; }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_warning(height: str = '') -> str:
    """Amber/orange button — used for active teleop state."""
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background-color: #d97706; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 10px 20px;
            font-weight: 600; font-size: 14px; {h}
        }}
        QPushButton:hover {{ background-color: #b45309; }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_ghost() -> str:
    return f"""
        QPushButton {{
            background-color: {BG_CARD}; color: {TEXT_BODY};
            border: 1px solid {BORDER}; border-radius: {RADIUS_MD}; padding: 10px 20px;
            font-size: 14px;
        }}
        QPushButton:hover {{ background-color: {BG_SURFACE}; }}
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
    """Small square icon button (refresh, etc.)."""
    return f"""
        QPushButton {{
            background-color: {BG_CARD}; border: 1px solid {BORDER};
            border-radius: {RADIUS_SM}; color: {TEXT_BODY}; font-size: 14px;
        }}
        QPushButton:hover {{ background-color: {BG_SURFACE}; }}
    """


def groupbox_style() -> str:
    return f"""
        QGroupBox {{
            color: {TEXT_MUTED}; font-size: 11px; font-weight: 600;
            letter-spacing: 0.5px; text-transform: uppercase;
            background-color: {BG_CARD};
            border: 1px solid {BORDER}; border-radius: {RADIUS_LG};
            margin-top: 10px; padding-top: 12px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin; left: 12px; padding: 0 4px;
        }}
    """


def combobox_style() -> str:
    return f"""
        QComboBox {{
            background-color: {BG_INPUT};
            border: 1px solid {BORDER};
            border-radius: {RADIUS_MD};
            padding: 6px 10px;
            color: {TEXT_H1};
            font-size: 14px;
        }}
        QComboBox:focus {{ border-color: {BORDER_FOCUS}; }}
        QComboBox::drop-down {{ border: none; width: 20px; }}
        QComboBox QAbstractItemView {{
            background-color: {BG_CARD};
            color: {TEXT_BODY};
            selection-background-color: {ACCENT_ACTIVE_BG};
            border: 1px solid {BORDER};
            outline: none;
        }}
    """


def scrollbar_style() -> str:
    return f"""
        QScrollArea {{ border: none; background-color: transparent; }}
        QScrollBar:vertical {{
            background-color: {BG_SURFACE}; width: 8px; border-radius: 4px;
        }}
        QScrollBar::handle:vertical {{
            background-color: #c8cdd8; border-radius: 4px; min-height: 20px;
        }}
        QScrollBar::handle:vertical:hover {{
            background-color: {TEXT_DISABLED};
        }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}
    """


def progressbar_style(color: str) -> str:
    return f"""
        QProgressBar {{
            border: none; border-radius: {RADIUS_SM};
            background-color: {BG_SURFACE};
        }}
        QProgressBar::chunk {{
            background-color: {color};
            border-radius: {RADIUS_SM};
        }}
    """


def messagebox_style() -> str:
    return f"""
        QMessageBox {{ background-color: {BG_CARD}; color: {TEXT_H1}; }}
        QLabel {{ color: {TEXT_BODY}; font-size: 14px; }}
        QPushButton {{
            background-color: {ACCENT}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 8px 20px;
            font-weight: 600; min-width: 80px;
        }}
        QPushButton:hover {{ background-color: {ACCENT_HOVER}; }}
    """
