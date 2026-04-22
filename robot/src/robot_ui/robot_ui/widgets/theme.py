# Design tokens aligned with the web UI's light theme
BG_PAGE    = '#f5f5f5'
BG_CARD    = '#ffffff'
BG_INPUT   = '#ffffff'
BG_SIDEBAR = '#ffffff'

TEXT_H1       = '#111111'
TEXT_BODY     = '#333333'
TEXT_MUTED    = '#666666'
TEXT_DISABLED = '#9ca3af'

ACCENT             = '#4a90d9'
ACCENT_HOVER       = '#3a7bc8'
ACCENT_GREEN       = '#16a34a'
ACCENT_GREEN_HOVER = '#15803d'
ACCENT_RED         = '#dc2626'
ACCENT_RED_HOVER   = '#b91c1c'

BORDER       = '#e5e7eb'
BORDER_FOCUS = '#4a90d9'

RADIUS_LG = '12px'
RADIUS_MD  = '8px'
RADIUS_SM  = '4px'


def btn_primary(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background-color: {ACCENT}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 8px 20px;
            font-weight: 600; font-size: 13px; {h}
        }}
        QPushButton:hover {{ background-color: {ACCENT_HOVER}; }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_success(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background-color: {ACCENT_GREEN}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 8px 20px;
            font-weight: 600; font-size: 13px; {h}
        }}
        QPushButton:hover {{ background-color: {ACCENT_GREEN_HOVER}; }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_danger(height: str = '') -> str:
    h = f'min-height: {height};' if height else ''
    return f"""
        QPushButton {{
            background-color: {ACCENT_RED}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 8px 20px;
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
            border: none; border-radius: {RADIUS_MD}; padding: 8px 20px;
            font-weight: 600; font-size: 14px; {h}
        }}
        QPushButton:hover {{ background-color: #b45309; }}
        QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_DISABLED}; }}
    """


def btn_ghost() -> str:
    return f"""
        QPushButton {{
            background-color: {BG_CARD}; color: {TEXT_BODY};
            border: 1px solid {BORDER}; border-radius: {RADIUS_MD}; padding: 8px 20px;
            font-size: 13px;
        }}
        QPushButton:hover {{ background-color: #f9fafb; }}
    """


def btn_back() -> str:
    return f"""
        QPushButton {{
            background-color: transparent; color: {TEXT_MUTED};
            border: none; padding: 8px 4px; font-size: 12px;
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
        QPushButton:hover {{ background-color: #f9fafb; }}
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
            padding: 4px 8px;
            color: {TEXT_H1};
            font-size: 13px;
        }}
        QComboBox:focus {{ border-color: {BORDER_FOCUS}; }}
        QComboBox::drop-down {{ border: none; width: 20px; }}
        QComboBox QAbstractItemView {{
            background-color: {BG_CARD};
            color: {TEXT_BODY};
            selection-background-color: #eff6ff;
            border: 1px solid {BORDER};
            outline: none;
        }}
    """


def scrollbar_style() -> str:
    return f"""
        QScrollArea {{ border: none; background-color: transparent; }}
        QScrollBar:vertical {{
            background-color: #f3f4f6; width: 10px; border-radius: 5px;
        }}
        QScrollBar::handle:vertical {{
            background-color: #d1d5db; border-radius: 5px; min-height: 20px;
        }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}
    """


def progressbar_style(color: str) -> str:
    return f"""
        QProgressBar {{
            border: none; border-radius: {RADIUS_SM};
            background-color: #f3f4f6;
        }}
        QProgressBar::chunk {{
            background-color: {color};
            border-radius: {RADIUS_SM};
        }}
    """


def messagebox_style() -> str:
    return f"""
        QMessageBox {{ background-color: {BG_CARD}; color: {TEXT_H1}; }}
        QLabel {{ color: {TEXT_BODY}; font-size: 13px; }}
        QPushButton {{
            background-color: {ACCENT}; color: white;
            border: none; border-radius: {RADIUS_MD}; padding: 6px 20px;
            font-weight: 600; min-width: 80px;
        }}
        QPushButton:hover {{ background-color: {ACCENT_HOVER}; }}
    """
