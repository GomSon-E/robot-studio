import threading
from pathlib import Path

import serial.tools.list_ports
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QGridLayout, QFrame, QStackedWidget, QMessageBox,
)
from PySide6.QtCore import Qt, Signal, QObject, QPoint, QSize
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygon, QPixmap, QMovie

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .theme import (
    GLASS_BG, GLASS_BORDER, RADIUS_LG, RADIUS_MD,
    TEXT_H1, TEXT_BODY, TEXT_MUTED, TEXT_DISABLED,
    ACCENT, ACCENT_GREEN, ACCENT_RED,
    btn_primary, btn_success, btn_ghost, btn_back, btn_icon_sm,
    combobox_style, messagebox_style,
)

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
RESOLUTION = 4096
HALF_TURN = RESOLUTION // 2

ASSETS_DIR = Path(__file__).parent.parent / 'assets' / 'calibration'

_BTN_PRIMARY = btn_primary()
_BTN_GREEN   = btn_success()
_BTN_GHOST   = btn_ghost()
_BTN_BACK    = btn_back()

_LED_STYLE = {
    'idle':       'color: #94a3b8; font-size: 14px; background: transparent; border: none;',
    'connecting': 'color: #f59e0b; font-size: 14px; background: transparent; border: none;',
    'connected':  'color: #059669; font-size: 14px; background: transparent; border: none;',
}


# ─── ROS2 → Qt 시그널 ────────────────────────────────────────────────────────

class CalibrationSignals(QObject):
    status_received = Signal(str, str, dict)
    joints_received = Signal(list)
    ranges_received = Signal(dict)


# ─── UI 전용 ROS2 노드 ────────────────────────────────────────────────────────

class CalibrationUINode(Node):
    def __init__(self, signals: CalibrationSignals):
        super().__init__('calibration_ui_node')
        self._signals = signals
        self.create_subscription(String, '/calibration/status', self._on_status, 10)
        self.create_subscription(JointState, '/calibration/joint_states', self._on_joints, 10)
        self.create_subscription(String, '/calibration/tracked_ranges', self._on_ranges, 10)
        self._cmd_pub = self.create_publisher(String, '/calibration/command', 10)

    def _on_status(self, msg: String):
        import json
        try:
            data = json.loads(msg.data)
            self._signals.status_received.emit(
                data.get('status', ''),
                data.get('message', ''),
                data.get('data', {}),
            )
        except json.JSONDecodeError:
            pass

    def _on_joints(self, msg: JointState):
        self._signals.joints_received.emit(list(msg.position))

    def _on_ranges(self, msg: String):
        import json
        try:
            self._signals.ranges_received.emit(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def send_command(self, command: str):
        self._cmd_pub.publish(String(data=command))


# ─── 관절 슬라이더 위젯 ──────────────────────────────────────────────────────

class JointRangeSlider(QWidget):
    """
    하나의 관절에 대한 범위 슬라이더.
    빨간색 전체 바, 녹색 활성 구간(자동 추적), 노란색 현재 위치 tick.
    """

    _font_label = QFont('monospace', 9)
    _font_name = QFont('sans-serif', 11, QFont.Weight.Bold)

    def __init__(self, joint_name: str, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.resolution = RESOLUTION
        self.min_v = HALF_TURN
        self.max_v = HALF_TURN
        self.pos_v = HALF_TURN
        self.tick_v = HALF_TURN
        self.setMinimumHeight(60)
        self.setFixedHeight(80)

    def set_range(self, min_val: int, max_val: int, pos_val: int):
        new_min = max(0, min(RESOLUTION - 1, min_val))
        new_max = max(0, min(RESOLUTION - 1, max_val))
        new_pos = max(0, min(RESOLUTION - 1, pos_val))
        if new_min == self.min_v and new_max == self.max_v and new_pos == self.pos_v:
            return
        self.min_v, self.max_v, self.pos_v = new_min, new_max, new_pos
        self.update()

    def set_tick(self, tick: int):
        new_tick = max(0, min(RESOLUTION - 1, tick))
        if new_tick == self.tick_v:
            return
        self.tick_v = new_tick
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        margin = 56
        bar_y = h // 2 + 4
        bar_w = w - margin * 2
        bar_h = 8
        x0 = margin

        def val_to_x(v):
            return x0 + (v / (RESOLUTION - 1)) * bar_w

        # 배경 트랙 (중립 회색)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(226, 229, 236))
        painter.drawRoundedRect(x0, bar_y - bar_h // 2, bar_w, bar_h, 4, 4)

        # 캘리브레이션 구간 (퍼플)
        active_x = int(val_to_x(self.min_v))
        active_w = int(val_to_x(self.max_v) - val_to_x(self.min_v))
        if active_w > 0:
            painter.setBrush(QColor(196, 181, 253))
            painter.drawRoundedRect(active_x, bar_y - bar_h // 2, active_w, bar_h, 4, 4)

        # min/max 핸들
        painter.setPen(QPen(QColor(71, 85, 105), 2))
        for x in [val_to_x(self.min_v), val_to_x(self.max_v)]:
            painter.drawLine(int(x), bar_y - 10, int(x), bar_y + 10)

        # 현재 위치 tick (중간 회색)
        painter.setPen(QPen(QColor(148, 163, 184), 2))
        tx = int(val_to_x(self.tick_v))
        painter.drawLine(tx, bar_y - 12, tx, bar_y + 12)

        # 삼각형 (live pos) — 퍼플
        painter.setBrush(QColor(124, 58, 237))
        painter.setPen(Qt.PenStyle.NoPen)
        px = int(val_to_x(self.pos_v))
        tri = QPolygon([
            QPoint(px, bar_y - 14),
            QPoint(px - 6, bar_y - 24),
            QPoint(px + 6, bar_y - 24),
        ])
        painter.drawPolygon(tri)

        # 숫자 라벨
        painter.setPen(QColor(100, 116, 139))
        painter.setFont(self._font_label)
        painter.drawText(int(val_to_x(self.min_v)) - 15, bar_y - 30, 30, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.min_v))
        painter.drawText(int(val_to_x(self.max_v)) - 15, bar_y - 30, 30, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.max_v))
        painter.drawText(px - 20, bar_y - 34, 40, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.pos_v))

        # 관절 이름
        painter.setPen(QColor(26, 29, 46))
        painter.setFont(self._font_name)
        painter.drawText(8, 6, w - 16, 18, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                         self.joint_name)

        painter.end()


# ─── 메인 패널 ───────────────────────────────────────────────────────────────

class CalibrationPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._ui_node: CalibrationUINode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._ros_thread: threading.Thread | None = None
        self._running = False

        self._connected = False
        self._cal_status = 'idle'
        self._current_positions = [float(HALF_TURN)] * 6
        self._current_step = 0

        self._sliders: dict[str, JointRangeSlider] = {}
        self._step_labels: list[QLabel] = []

        self._signals = CalibrationSignals()
        self._signals.status_received.connect(self._on_status)
        self._signals.joints_received.connect(self._on_joints)
        self._signals.ranges_received.connect(self._on_ranges)

        self._setup_ui()
        self._init_ros()

    # ─── UI 빌더 헬퍼 ─────────────────────────────────────────────────────────

    def _make_instruction_card(self, title: str, desc: str) -> QFrame:
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{
                background-color: rgba(245, 243, 255, 0.85);
                border-radius: {RADIUS_MD};
                border: 1px solid rgba(196, 181, 253, 0.5);
            }}
        """)
        layout = QVBoxLayout(card)
        layout.setSpacing(4)
        layout.setContentsMargins(12, 10, 12, 10)

        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(
            'color: #5b21b6; font-size: 14px; font-weight: 600; background: transparent; border: none;'
        )
        layout.addWidget(title_lbl)

        desc_lbl = QLabel(desc)
        desc_lbl.setStyleSheet(f'color: {TEXT_BODY}; font-size: 13px; background: transparent; border: none;')
        desc_lbl.setWordWrap(True)
        layout.addWidget(desc_lbl)

        return card

    def _make_image_label(self, filename: str, fallback_text: str, height: int = 180) -> QLabel:
        img_path = ASSETS_DIR / filename
        label = QLabel()
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setFixedHeight(height)
        if img_path.exists():
            pixmap = QPixmap(str(img_path))
            label.setPixmap(pixmap.scaled(
                600, height,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            ))
            label.setStyleSheet(
                'background: rgba(196,181,253,0.12); border-radius: 8px;'
                ' border: 1px solid rgba(196,181,253,0.25);'
            )
        else:
            label.setText(fallback_text)
            label.setStyleSheet(
                f'background-color: rgba(196,181,253,0.15); border-radius: 8px;'
                f' border: 1px solid rgba(196,181,253,0.3);'
                f' color: {TEXT_DISABLED}; font-size: 13px;'
            )
            label.setWordWrap(True)
        return label

    def _make_gif_player(self, filename: str, height: int = 180) -> QLabel:
        gif_path = ASSETS_DIR / filename
        label = QLabel()
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setFixedHeight(height)
        if gif_path.exists():
            movie = QMovie(str(gif_path))
            movie.setScaledSize(QSize(height * 4 // 3, height))
            label.setMovie(movie)
            movie.start()
            label.setStyleSheet(
                'background: rgba(196,181,253,0.12); border-radius: 8px;'
                ' border: 1px solid rgba(196,181,253,0.25);'
            )
        else:
            label.setText(filename)
            label.setStyleSheet(
                f'background-color: rgba(196,181,253,0.15); border-radius: 8px;'
                f' border: 1px solid rgba(196,181,253,0.3);'
                f' color: {TEXT_DISABLED}; font-size: 13px;'
            )
        return label

    # ─── UI 설정 ──────────────────────────────────────────────────────────────

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(6)

        title = QLabel('Robot Calibration')
        title.setStyleSheet(f'color: {TEXT_H1}; font-size: 22px; font-weight: 700; background: transparent;')
        main_layout.addWidget(title)

        main_layout.addWidget(self._build_step_indicator())

        self._stacked = QStackedWidget()
        self._stacked.addWidget(self._build_page_connect())   # 0
        self._stacked.addWidget(self._build_page_center())    # 1
        self._stacked.addWidget(self._build_page_explore())   # 2
        main_layout.addWidget(self._stacked, 1)

        self._go_to_step(0)
        self._refresh_ports()

    def _build_step_indicator(self) -> QWidget:
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: none;
                border-radius: {RADIUS_MD};
            }}
        """)
        outer = QVBoxLayout(card)
        outer.setContentsMargins(16, 10, 16, 10)
        outer.setSpacing(0)

        widget = QWidget()
        widget.setStyleSheet("background: transparent;")
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        steps = ['① 연결', '② 중앙 위치', '③ 범위 탐색']
        self._step_labels = []

        for i, text in enumerate(steps):
            lbl = QLabel(text)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setFixedHeight(28)
            lbl.setContentsMargins(4, 0, 4, 0)
            self._step_labels.append(lbl)
            layout.addWidget(lbl, 2)

            if i < len(steps) - 1:
                sep = QLabel('›')
                sep.setAlignment(Qt.AlignmentFlag.AlignCenter)
                sep.setStyleSheet('color: rgba(196,181,253,0.6); font-size: 16px; background: transparent;')
                sep.setFixedWidth(20)
                layout.addWidget(sep)

        outer.addWidget(widget)
        return card

    def _update_step_indicator(self):
        for i, lbl in enumerate(self._step_labels):
            if i < self._current_step:
                lbl.setStyleSheet(
                    f'color: {ACCENT_GREEN}; font-size: 12px; background: transparent; padding: 2px 4px;'
                )
            elif i == self._current_step:
                lbl.setStyleSheet(
                    f'color: white; font-size: 12px; font-weight: 700; '
                    f'background: {ACCENT}; border-radius: 6px; padding: 2px 4px;'
                )
            else:
                lbl.setStyleSheet(
                    f'color: {TEXT_DISABLED}; font-size: 12px; background: transparent; padding: 2px 4px;'
                )

    def _go_to_step(self, step: int):
        self._current_step = step
        self._stacked.setCurrentIndex(step)
        self._update_step_indicator()

    # ─── 페이지 빌더 ──────────────────────────────────────────────────────────

    def _build_page_connect(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(16)
        layout.setContentsMargins(0, 0, 0, 8)

        layout.addWidget(self._make_instruction_card(
            '로봇을 연결하세요',
            '시리얼 포트를 선택하고 연결을 클릭하세요.',
        ))

        conn_card = QFrame()
        conn_card.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: none;
                border-radius: {RADIUS_LG};
            }}
        """)
        conn_card_layout = QVBoxLayout(conn_card)
        conn_card_layout.setContentsMargins(12, 12, 12, 12)
        conn_card_layout.setSpacing(0)

        conn_label = QLabel('연결 설정')
        conn_label.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 13px; font-weight: 600; background: transparent; border: none;'
        )
        conn_card_layout.addWidget(conn_label)

        inner = QWidget()
        inner.setStyleSheet('background: transparent;')
        inner_layout = QHBoxLayout(inner)
        inner_layout.setContentsMargins(0, 8, 0, 0)
        inner_layout.setSpacing(8)

        self._conn_led = QLabel('●')
        self._conn_led.setFixedWidth(20)
        self._conn_led.setStyleSheet(_LED_STYLE['idle'])
        inner_layout.addWidget(self._conn_led)

        self._conn_status_lbl = QLabel('대기')
        self._conn_status_lbl.setFixedWidth(80)
        self._conn_status_lbl.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 13px; background: transparent; border: none;'
        )
        inner_layout.addWidget(self._conn_status_lbl)

        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(160)
        self._port_combo.setStyleSheet(combobox_style())
        inner_layout.addWidget(self._port_combo)

        refresh_btn = QPushButton('↺')
        refresh_btn.setFixedSize(30, 30)
        refresh_btn.setStyleSheet(btn_icon_sm())
        refresh_btn.clicked.connect(self._refresh_ports)
        inner_layout.addWidget(refresh_btn)

        self._role_combo = QComboBox()
        self._role_combo.addItems(['팔로워암', '리더암'])
        self._role_combo.setFixedWidth(100)
        self._role_combo.setStyleSheet(combobox_style())
        inner_layout.addWidget(self._role_combo)

        self._connect_btn = QPushButton('연결')
        self._connect_btn.setFixedHeight(34)
        self._connect_btn.setStyleSheet(_BTN_PRIMARY)
        self._connect_btn.clicked.connect(self._on_connect)
        inner_layout.addWidget(self._connect_btn)

        inner_layout.addStretch()
        conn_card_layout.addWidget(inner)

        layout.addWidget(conn_card)
        layout.addStretch()

        return page

    def _build_page_center(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(12)
        layout.setContentsMargins(0, 8, 0, 8)

        layout.addWidget(self._make_instruction_card(
            '모든 관절을 중앙 위치로 맞춰주세요',
            '아래 사진처럼 로봇 팔의 모든 관절을 중앙(중립) 위치로 손으로 맞추세요.\n'
            '완료되면 "캘리브레이션 시작"을 클릭합니다. 클릭 순간의 위치가 기준점으로 기록됩니다.',
        ))

        layout.addWidget(self._make_gif_player('0.gif', height=360))

        layout.addStretch()

        bottom = QHBoxLayout()
        back_btn = QPushButton('← 돌아가기')
        back_btn.setStyleSheet(_BTN_BACK)
        back_btn.clicked.connect(self._on_back_to_connect)
        bottom.addWidget(back_btn)
        bottom.addStretch()
        start_btn = QPushButton('캘리브레이션 시작')
        start_btn.setFixedHeight(44)
        start_btn.setStyleSheet(_BTN_GREEN)
        start_btn.clicked.connect(self._on_start)
        bottom.addWidget(start_btn)
        layout.addLayout(bottom)

        return page

    def _build_page_explore(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setSpacing(8)
        layout.setContentsMargins(0, 8, 0, 8)

        layout.addWidget(self._make_instruction_card(
            '각 관절을 끝까지 당기고 밀어주세요',
            '토크가 꺼진 상태입니다. 각 관절을 손으로 최소 위치까지, 그다음 최대 위치까지 움직여 주세요.\n'
            '슬라이더의 녹색 구간이 자동으로 넓어집니다. 완료되면 저장 버튼을 클릭하세요.',
        ))

        # 3열 그리드 — GIF + 슬라이더 6개 동시 표시
        grid_widget = QWidget()
        grid_widget.setStyleSheet('background: transparent;')
        grid_layout = QGridLayout(grid_widget)
        grid_layout.setSpacing(8)
        grid_layout.setContentsMargins(0, 4, 0, 4)

        for i, name in enumerate(JOINT_NAMES):
            cell = QWidget()
            cell.setStyleSheet('background: transparent;')
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(4)

            cell_layout.addWidget(self._make_gif_player(f'{i + 1}.gif', height=140))

            slider = JointRangeSlider(name)
            self._sliders[name] = slider
            cell_layout.addWidget(slider)

            row, col = divmod(i, 3)
            grid_layout.addWidget(cell, row, col)

        grid_card = QFrame()
        grid_card.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: 1px solid {GLASS_BORDER};
                border-radius: {RADIUS_LG};
            }}
        """)
        grid_card_layout = QVBoxLayout(grid_card)
        grid_card_layout.setContentsMargins(12, 12, 12, 12)
        grid_card_layout.addWidget(grid_widget)
        layout.addWidget(grid_card)
        layout.addStretch()

        bottom = QHBoxLayout()
        back_btn = QPushButton('← 돌아가기')
        back_btn.setStyleSheet(_BTN_BACK)
        back_btn.clicked.connect(self._on_back_to_center)
        bottom.addWidget(back_btn)
        bottom.addStretch()
        save_btn = QPushButton('범위 확정 및 저장')
        save_btn.setFixedHeight(44)
        save_btn.setStyleSheet(_BTN_PRIMARY)
        save_btn.clicked.connect(self._on_save)
        bottom.addWidget(save_btn)
        layout.addLayout(bottom)

        return page

# ─── ROS2 초기화 ──────────────────────────────────────────────────────────

    def _init_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self._executor = MultiThreadedExecutor()
            self._ui_node = CalibrationUINode(self._signals)
            self._executor.add_node(self._ui_node)
            self._running = True
            self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self._ros_thread.start()
        except Exception:
            pass

    def _ros_spin(self):
        while self._running and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    # ─── UI 핸들러 ────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        self._port_combo.clear()
        for p in serial.tools.list_ports.comports():
            if '/dev/ttyS' not in p.device:
                self._port_combo.addItem(f'{p.device} — {p.description}', p.device)
        if self._port_combo.count() == 0:
            self._port_combo.addItem('No ports', '')

    def _on_connect(self):
        port = self._port_combo.currentData()
        if not port:
            return
        role_text = self._role_combo.currentText()
        arm_role = 'follower' if role_text == '팔로워암' else 'leader'
        self._conn_led.setStyleSheet(_LED_STYLE['connecting'])
        self._conn_status_lbl.setText('연결 중...')
        self._connect_btn.setEnabled(False)
        self._port_combo.setEnabled(False)
        self._role_combo.setEnabled(False)
        if self._ui_node:
            self._ui_node.send_command(f'connect {port} {arm_role}')

    def _on_start(self):
        if self._ui_node:
            self._ui_node.send_command('start')

    def _on_save(self):
        unmoved = [
            name for name, slider in self._sliders.items()
            if slider.min_v == slider.max_v
        ]
        if unmoved:
            dlg = QMessageBox(self)
            dlg.setWindowTitle('범위 탐색 미완료')
            dlg.setText(
                '아직 움직이지 않은 관절이 있습니다:\n' +
                ', '.join(unmoved) +
                '\n\n각 관절을 최소·최대 위치까지 움직인 뒤 저장해 주세요.'
            )
            dlg.setStandardButtons(QMessageBox.StandardButton.Ok)
            dlg.setStyleSheet(messagebox_style())
            dlg.exec()
            return
        if self._ui_node:
            self._ui_node.send_command('save')

    def _on_back_to_connect(self):
        if self._ui_node:
            self._ui_node.send_command('disconnect')

    def _on_back_to_center(self):
        self._go_to_step(1)

# ─── ROS2 콜백 ────────────────────────────────────────────────────────────

    def _on_status(self, status: str, message: str, data: dict):
        self._cal_status = status

        if status == 'connected':
            self._connected = True
            self._conn_led.setStyleSheet(_LED_STYLE['connected'])
            self._conn_status_lbl.setText('연결됨')
            self._connect_btn.setEnabled(True)
            self._port_combo.setEnabled(True)
            self._role_combo.setEnabled(True)
            self._go_to_step(1)
        elif status == 'idle':
            self._connected = False
            self._conn_led.setStyleSheet(_LED_STYLE['idle'])
            self._conn_status_lbl.setText('대기')
            self._connect_btn.setEnabled(True)
            self._port_combo.setEnabled(True)
            self._role_combo.setEnabled(True)
            self._go_to_step(0)
        elif status == 'error':
            self._conn_led.setStyleSheet(_LED_STYLE['idle'])
            self._conn_status_lbl.setText('오류')
            self._connect_btn.setEnabled(True)
            self._port_combo.setEnabled(True)
            self._role_combo.setEnabled(True)
        elif status == 'homing_set':
            for slider in self._sliders.values():
                slider.set_range(HALF_TURN, HALF_TURN, HALF_TURN)
            self._go_to_step(2)
        elif status == 'saved':
            file_path = data.get('file_path', '')
            dlg = QMessageBox(self)
            dlg.setWindowTitle('캘리브레이션 저장 완료')
            dlg.setText(f'캘리브레이션이 저장되었습니다\n{file_path}')
            dlg.setStandardButtons(QMessageBox.StandardButton.Ok)
            dlg.setStyleSheet(messagebox_style())
            dlg.exec()
            self._go_to_step(0)

    def _on_joints(self, positions: list):
        self._current_positions = positions
        for i, name in enumerate(JOINT_NAMES):
            if i < len(positions) and name in self._sliders:
                self._sliders[name].set_tick(int(positions[i]))

    def _on_ranges(self, ranges: dict):
        for name in JOINT_NAMES:
            if name in ranges and name in self._sliders:
                r = ranges[name]
                slider = self._sliders[name]
                slider.set_range(r['min'], r['max'], slider.tick_v)

    def cleanup(self):
        self._running = False
        if self._ros_thread and self._ros_thread.is_alive():
            self._ros_thread.join(timeout=1.0)
        if self._executor:
            self._executor.shutdown()
        if self._ui_node is not None:
            self._ui_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
