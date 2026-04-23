import re
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSpinBox, QDoubleSpinBox,
    QFrame, QPushButton, QLineEdit, QComboBox
)
from PySide6.QtCore import Qt, Signal

from .theme import (
    GLASS_BG, GLASS_BORDER, TEXT_H1, TEXT_BODY, BORDER_FOCUS,
    RADIUS_MD, btn_primary,
)

_COMBO_STYLE = f"""
    QComboBox {{
        background-color: {GLASS_BG};
        border: none;
        border-radius: {RADIUS_MD};
        padding: 6px 10px; color: {TEXT_H1}; font-size: 14px;
    }}
    QComboBox::drop-down {{ border: none; width: 20px; }}
    QComboBox QAbstractItemView {{
        background-color: white; color: {TEXT_BODY};
        selection-background-color: rgba(124,58,237,0.1);
        border: none; outline: none;
    }}
"""

CAMERA_ROLES = ['top', 'wrist']

LABEL_STYLE = f"color: {TEXT_BODY}; font-size: 14px; background: transparent;"
INPUT_STYLE = f"""
    background-color: {GLASS_BG};
    color: {TEXT_H1};
    border: none;
    border-radius: {RADIUS_MD};
    padding: 6px 12px;
    font-size: 14px;
"""
FOCUSED_BORDER = f"border-color: {BORDER_FOCUS};"


class DatasetSettingPanel(QWidget):
    """데이터셋 설정 패널"""

    submitted = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._camera_combos: dict[str, QComboBox] = {}
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        title = QLabel('Dataset Setting')
        title.setStyleSheet(f"color: {TEXT_H1}; font-size: 22px; font-weight: 700; background: transparent;")
        layout.addWidget(title)

        settings_card = QFrame()
        settings_card.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: none;
                border-radius: 14px;
            }}
        """)
        settings_inner = QVBoxLayout(settings_card)
        settings_inner.setContentsMargins(20, 16, 20, 16)
        settings_inner.setSpacing(12)
        settings_layout = settings_inner

        self.dataset_name_edit = self._create_lineedit_row(
            settings_layout, 'Dataset Name',
            datetime.now().strftime("dataset_%Y%m%d_%H%M%S"),
        )

        for role in CAMERA_ROLES:
            combo = self._create_combo_row(settings_layout, f'Camera: {role}')
            self._camera_combos[role] = combo

        self.language_edit = self._create_lineedit_row(
            settings_layout, 'Language Instruction', ''
        )

        self.episode_spin = self._create_spin_row(
            settings_layout, 'Episodes', 1, 1000, 10
        )

        self.data_length_spin = self._create_double_spin_row(
            settings_layout, 'Data Length (sec)', 0.1, 3600.0, 10.0
        )

        self.term_length_spin = self._create_double_spin_row(
            settings_layout, 'Term Length (sec)', 0.0, 3600.0, 1.0
        )

        layout.addWidget(settings_card)
        layout.addStretch()

        btn_layout = QHBoxLayout()
        self.submit_btn = QPushButton('Start Recording')
        self.submit_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.submit_btn.setStyleSheet(btn_primary())
        self.submit_btn.clicked.connect(self._on_submit)
        btn_layout.addWidget(self.submit_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    # ------------------------------------------------------------------
    # 위젯 팩토리
    # ------------------------------------------------------------------

    def _create_lineedit_row(
        self, parent_layout: QVBoxLayout, label: str, default: str
    ) -> QLineEdit:
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setStyleSheet(LABEL_STYLE)
        lbl.setFixedWidth(160)
        row.addWidget(lbl)

        edit = QLineEdit(default)
        edit.setStyleSheet(f"QLineEdit {{ {INPUT_STYLE} }} QLineEdit:focus {{ {FOCUSED_BORDER} }}")
        row.addWidget(edit, 1)
        parent_layout.addLayout(row)
        return edit

    def _create_combo_row(
        self, parent_layout: QVBoxLayout, label: str
    ) -> QComboBox:
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setStyleSheet(LABEL_STYLE)
        lbl.setFixedWidth(160)
        row.addWidget(lbl)

        combo = QComboBox()
        combo.addItem('')
        combo.setStyleSheet(_COMBO_STYLE)
        row.addWidget(combo, 1)
        parent_layout.addLayout(row)
        return combo

    def _create_spin_row(
        self, parent_layout: QVBoxLayout, label: str,
        min_val: int, max_val: int, default: int
    ) -> QSpinBox:
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setStyleSheet(LABEL_STYLE)
        lbl.setFixedWidth(160)
        row.addWidget(lbl)

        spin = QSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(default)
        spin.setStyleSheet(f"QSpinBox {{ {INPUT_STYLE} }} QSpinBox:focus {{ {FOCUSED_BORDER} }}")
        spin.setFixedWidth(160)
        row.addWidget(spin)
        row.addStretch()
        parent_layout.addLayout(row)
        return spin

    def _create_double_spin_row(
        self, parent_layout: QVBoxLayout, label: str,
        min_val: float, max_val: float, default: float
    ) -> QDoubleSpinBox:
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setStyleSheet(LABEL_STYLE)
        lbl.setFixedWidth(160)
        row.addWidget(lbl)

        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(default)
        spin.setDecimals(1)
        spin.setSingleStep(0.5)
        spin.setStyleSheet(f"QDoubleSpinBox {{ {INPUT_STYLE} }} QDoubleSpinBox:focus {{ {FOCUSED_BORDER} }}")
        spin.setFixedWidth(160)
        row.addWidget(spin)
        row.addStretch()
        parent_layout.addLayout(row)
        return spin

    # ------------------------------------------------------------------
    # 공개 API
    # ------------------------------------------------------------------

    _AUTO_NAME_RE = re.compile(r'^dataset_\d{8}(_\d{6})?$')

    def showEvent(self, event):
        super().showEvent(event)
        current = self.dataset_name_edit.text().strip()
        if not current or self._AUTO_NAME_RE.fullmatch(current):
            self.dataset_name_edit.setText(datetime.now().strftime("dataset_%Y%m%d_%H%M%S"))

    def set_available_topics(self, topics: list[str]):
        for combo in self._camera_combos.values():
            current = combo.currentText()
            combo.clear()
            combo.addItem('')
            for t in topics:
                combo.addItem(t)
            idx = combo.findText(current)
            combo.setCurrentIndex(idx if idx >= 0 else 0)

    def get_settings(self) -> dict:
        camera_roles = {}
        for role, combo in self._camera_combos.items():
            topic = combo.currentText()
            if topic:
                camera_roles[role] = topic

        return {
            'dataset_name':         self.dataset_name_edit.text().strip(),
            'camera_roles':         camera_roles,
            'language_instruction': self.language_edit.text().strip(),
            'episodes':             self.episode_spin.value(),
            'data_length':          self.data_length_spin.value(),
            'term_length':          self.term_length_spin.value(),
        }

    def _on_submit(self):
        self.submitted.emit(self.get_settings())
