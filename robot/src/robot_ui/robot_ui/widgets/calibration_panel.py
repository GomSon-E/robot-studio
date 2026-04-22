import threading
from pathlib import Path

import serial.tools.list_ports
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QComboBox, QGridLayout, QFrame, QStackedWidget, QMessageBox,
)
from PySide6.QtCore import Qt, Signal, QObject
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygon, QPoint, QPixmap

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .theme import (
    BG_PAGE, BG_CARD, TEXT_H1, TEXT_BODY, TEXT_MUTED, TEXT_DISABLED,
    ACCENT, ACCENT_GREEN, ACCENT_RED, BORDER,
    btn_primary, btn_success, btn_ghost, btn_back, btn_icon_sm,
    groupbox_style, combobox_style, messagebox_style,
)

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
RESOLUTION = 4096
HALF_TURN = RESOLUTION // 2

ASSETS_DIR = Path(__file__).parent.parent / 'assets' / 'calibration'

_BTN_PRIMARY = btn_primary()
_BTN_GREEN   = btn_success()
_BTN_GHOST   = btn_ghost()
_BTN_BACK    = btn_back()


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
        self.min_v = max(0, min(RESOLUTION - 1, min_val))
        self.max_v = max(0, min(RESOLUTION - 1, max_val))
        self.pos_v = max(0, min(RESOLUTION - 1, pos_val))
        self.update()

    def set_tick(self, tick: int):
        self.tick_v = max(0, min(RESOLUTION - 1, tick))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # 카드 배경
        painter.fillRect(self.rect(), QColor(255, 255, 255))

        margin = 56
        bar_y = h // 2 + 4
        bar_w = w - margin * 2
        bar_h = 8
        x0 = margin

        def val_to_x(v):
            return x0 + (v / (RESOLUTION - 1)) * bar_w

        # 배경 바 (빨간색)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(239, 68, 68))
        painter.drawRoundedRect(x0, bar_y - bar_h // 2, bar_w, bar_h, 4, 4)

        # 활성 구간 (초록)
        active_x = int(val_to_x(self.min_v))
        active_w = int(val_to_x(self.max_v) - val_to_x(self.min_v))
        if active_w > 0:
            painter.setBrush(QColor(34, 197, 94))
            painter.drawRoundedRect(active_x, bar_y - bar_h // 2, active_w, bar_h, 4, 4)

        # min/max 핸들 (어두운 세로선)
        painter.setPen(QPen(QColor(55, 65, 81), 2))
        for x in [val_to_x(self.min_v), val_to_x(self.max_v)]:
            painter.drawLine(int(x), bar_y - 10, int(x), bar_y + 10)

        # 현재 위치 tick (황색 세로선)
        painter.setPen(QPen(QColor(234, 179, 8), 2))
        tx = int(val_to_x(self.tick_v))
        painter.drawLine(tx, bar_y - 12, tx, bar_y + 12)

        # 삼각형 (pos_v) — 슬레이트색
        painter.setBrush(QColor(71, 85, 105))
        painter.setPen(Qt.PenStyle.NoPen)
        px = int(val_to_x(self.pos_v))
        tri = QPolygon([
            QPoint(px, bar_y - 14),
            QPoint(px - 6, bar_y - 24),
            QPoint(px + 6, bar_y - 24),
        ])
        painter.drawPolygon(tri)

        # 숫자 라벨
        painter.setPen(QColor(71, 85, 105))
        font = QFont('monospace', 8)
        painter.setFont(font)
        painter.drawText(int(val_to_x(self.min_v)) - 15, bar_y - 30, 30, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.min_v))
        painter.drawText(int(val_to_x(self.max_v)) - 15, bar_y - 30, 30, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.max_v))
        painter.drawText(px - 20, bar_y - 34, 40, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.pos_v))

        # 관절 이름
        painter.setPen(QColor(51, 65, 85))
        painter.setFont(QFont('sans-serif', 10, QFont.Weight.Bold))
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
        card.setStyleSheet(f'QFrame {{ background: #eff6ff; border-radius: 8px; border: 1px solid #bfdbfe; }}')
        layout = QVBoxLayout(card)
        layout.setSpacing(4)
        layout.setContentsMargins(12, 10, 12, 10)

        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(
            'color: #1d4ed8; font-size: 13px; font-weight: 600; background: transparent; border: none;'
        )
        layout.addWidget(title_lbl)

        desc_lbl = QLabel(desc)
        desc_lbl.setStyleSheet(f'color: {TEXT_BODY}; font-size: 12px; background: transparent; border: none;')
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
            label.setStyleSheet(f'background: #f3f4f6; border-radius: 6px; border: 1px solid {BORDER};')
        else:
            label.setText(fallback_text)
            label.setStyleSheet(
                f'background: #f3f4f6; border-radius: 6px; color: {TEXT_DISABLED}; font-size: 12px; '
                f'border: 1px dashed {BORDER};'
            )
            label.setWordWrap(True)
        return label

    # ─── UI 설정 ──────────────────────────────────────────────────────────────

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(12)

        title = QLabel('Robot Calibration')
        title.setStyleSheet(f'color: {TEXT_H1}; font-size: 20px; font-weight: 700; background: transparent;')
        main_layout.addWidget(title)

        main_layout.addWidget(self._build_step_indicator())

        self._status_bar = QLabel('')
        self._status_bar.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 11px; padding: 2px 4px; background: transparent;')
        self._status_bar.setWordWrap(True)
        main_layout.addWidget(self._status_bar)

        self._stacked = QStackedWidget()
        self._stacked.addWidget(self._build_page_connect())   # 0
        self._stacked.addWidget(self._build_page_center())    # 1
        self._stacked.addWidget(self._build_page_explore())   # 2
        main_layout.addWidget(self._stacked, 1)

        self._go_to_step(0)
        self._refresh_ports()

    def _build_step_indicator(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(0)

        steps = ['① 연결', '② 중앙 위치', '③ 범위 탐색']
        self._step_labels = []

        for i, text in enumerate(steps):
            lbl = QLabel(text)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setFixedHeight(30)
            lbl.setContentsMargins(4, 0, 4, 0)
            self._step_labels.append(lbl)
            layout.addWidget(lbl, 2)

            if i < len(steps) - 1:
                sep = QLabel('›')
                sep.setAlignment(Qt.AlignmentFlag.AlignCenter)
                sep.setStyleSheet(f'color: {BORDER}; font-size: 16px; background: transparent;')
                sep.setFixedWidth(20)
                layout.addWidget(sep)

        return widget

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
        layout.setContentsMargins(0, 8, 0, 8)

        layout.addWidget(self._make_instruction_card(
            '로봇을 연결하세요',
            '시리얼 포트를 선택하고 Connect를 클릭하세요.',
        ))

        conn_group = QGroupBox('Connection')
        conn_group.setStyleSheet(groupbox_style())
        conn_layout = QHBoxLayout(conn_group)
        conn_layout.setSpacing(8)
        conn_layout.setContentsMargins(12, 18, 12, 12)

        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(160)
        self._port_combo.setStyleSheet(combobox_style())

        port_lbl = QLabel('Port:')
        port_lbl.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 12px; background: transparent; border: none;')
        conn_layout.addWidget(port_lbl)
        conn_layout.addWidget(self._port_combo)

        refresh_btn = QPushButton('↺')
        refresh_btn.setFixedSize(30, 30)
        refresh_btn.setStyleSheet(btn_icon_sm())
        refresh_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(refresh_btn)

        arm_lbl = QLabel('Arm:')
        arm_lbl.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 12px; background: transparent; border: none;')
        conn_layout.addWidget(arm_lbl)

        self._role_combo = QComboBox()
        self._role_combo.addItems(['follower', 'leader'])
        self._role_combo.setFixedWidth(100)
        self._role_combo.setStyleSheet(combobox_style())
        conn_layout.addWidget(self._role_combo)

        self._connect_btn = QPushButton('Connect')
        self._connect_btn.setFixedHeight(34)
        self._connect_btn.setStyleSheet(_BTN_PRIMARY)
        self._connect_btn.clicked.connect(self._on_connect)
        conn_layout.addWidget(self._connect_btn)

        conn_layout.addStretch()
        layout.addWidget(conn_group)
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

        layout.addWidget(
            self._make_image_label('center.jpg', '📷  참고 사진 준비 중\n로봇 팔 중앙 위치 모습', height=200)
        )

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

        layout.addWidget(
            self._make_image_label('explore.jpg', '📷  참고 사진 준비 중\n관절을 끝까지 돌리는 모습', height=100)
        )

        # 3열 그리드 — 스크롤 없이 한 화면에 표시
        grid_widget = QWidget()
        grid_widget.setStyleSheet('background: transparent;')
        grid_layout = QGridLayout(grid_widget)
        grid_layout.setSpacing(8)
        grid_layout.setContentsMargins(0, 4, 0, 4)

        for i, name in enumerate(JOINT_NAMES):
            slider = JointRangeSlider(name)
            self._sliders[name] = slider
            row, col = divmod(i, 3)
            grid_layout.addWidget(slider, row, col)

        layout.addWidget(grid_widget)
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
        arm_role = self._role_combo.currentText()
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

        if status == 'error':
            self._status_bar.setStyleSheet(
                f'color: {ACCENT_RED}; font-size: 11px; padding: 2px 4px; background: transparent;'
            )
            self._status_bar.setText(message)
            return

        self._status_bar.setStyleSheet(
            f'color: {TEXT_MUTED}; font-size: 11px; padding: 2px 4px; background: transparent;'
        )
        self._status_bar.setText(message)

        if status == 'connected':
            self._connected = True
            self._go_to_step(1)
        elif status == 'idle':
            self._connected = False
            self._go_to_step(0)
        elif status == 'homing_set':
            for slider in self._sliders.values():
                slider.set_range(HALF_TURN, HALF_TURN, HALF_TURN)
            self._go_to_step(2)
        elif status == 'saved':
            self._status_bar.setStyleSheet(
                f'color: {ACCENT_GREEN}; font-size: 11px; padding: 2px 4px; background: transparent;'
            )
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
