import threading
from pathlib import Path

import serial.tools.list_ports
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QComboBox, QScrollArea, QFrame, QStackedWidget, QMessageBox,
)
from PySide6.QtCore import Qt, Signal, QObject, QPoint
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygon, QPixmap

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
RESOLUTION = 4096
HALF_TURN = RESOLUTION // 2

ASSETS_DIR = Path(__file__).parent.parent / 'assets' / 'calibration'

_BTN_PRIMARY = """
    QPushButton {
        background-color: #0e639c; color: white;
        border: none; border-radius: 4px; padding: 8px 20px;
        font-weight: 600; font-size: 13px;
    }
    QPushButton:hover { background-color: #1177bb; }
    QPushButton:disabled { background-color: #2a2a2a; color: #555; }
"""
_BTN_GREEN = """
    QPushButton {
        background-color: #16825d; color: white;
        border: none; border-radius: 4px; padding: 8px 20px;
        font-weight: 600; font-size: 13px;
    }
    QPushButton:hover { background-color: #1a9a6e; }
    QPushButton:disabled { background-color: #2a2a2a; color: #555; }
"""
_BTN_GHOST = """
    QPushButton {
        background-color: #3c3c3c; color: #ccc;
        border: 1px solid #555; border-radius: 4px; padding: 8px 20px;
    }
    QPushButton:hover { background-color: #505050; }
"""
_BTN_BACK = """
    QPushButton {
        background-color: transparent; color: #666;
        border: none; padding: 8px 4px; font-size: 12px;
    }
    QPushButton:hover { color: #aaa; }
"""


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
        self.setFixedHeight(70)

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

        margin = 60
        bar_y = h // 2
        bar_w = w - margin * 2
        bar_h = 8
        x0 = margin

        def val_to_x(v):
            return x0 + (v / (RESOLUTION - 1)) * bar_w

        # 배경 바 (빨간색)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(200, 60, 60))
        painter.drawRect(x0, bar_y - bar_h // 2, bar_w, bar_h)

        # 활성 구간 (녹색) — min=max일 때는 0 너비
        active_x = int(val_to_x(self.min_v))
        active_w = int(val_to_x(self.max_v) - val_to_x(self.min_v))
        if active_w > 0:
            painter.setBrush(QColor(60, 200, 60))
            painter.drawRect(active_x, bar_y - bar_h // 2, active_w, bar_h)

        # min/max 핸들 (흰색 세로선)
        painter.setPen(QPen(QColor(240, 240, 240), 2))
        for x in [val_to_x(self.min_v), val_to_x(self.max_v)]:
            painter.drawLine(int(x), bar_y - 10, int(x), bar_y + 10)

        # 현재 위치 tick (노란색 세로선)
        painter.setPen(QPen(QColor(250, 220, 40), 2))
        tx = int(val_to_x(self.tick_v))
        painter.drawLine(tx, bar_y - 12, tx, bar_y + 12)

        # 삼각형 (pos_v)
        painter.setBrush(QColor(240, 240, 240))
        painter.setPen(Qt.PenStyle.NoPen)
        px = int(val_to_x(self.pos_v))
        tri = QPolygon([
            QPoint(px, bar_y - 14),
            QPoint(px - 6, bar_y - 24),
            QPoint(px + 6, bar_y - 24),
        ])
        painter.drawPolygon(tri)

        # 숫자 라벨
        painter.setPen(QColor(200, 200, 200))
        font = QFont('monospace', 9)
        painter.setFont(font)
        painter.drawText(int(val_to_x(self.min_v)) - 15, bar_y - 28, 30, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.min_v))
        painter.drawText(int(val_to_x(self.max_v)) - 15, bar_y - 28, 30, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.max_v))
        painter.drawText(px - 20, bar_y - 32, 40, 12,
                         Qt.AlignmentFlag.AlignCenter, str(self.pos_v))

        # 관절 이름
        painter.setPen(QColor(180, 180, 180))
        painter.setFont(QFont('sans-serif', 10, QFont.Weight.Bold))
        painter.drawText(4, 16, 100, 16, Qt.AlignmentFlag.AlignLeft, self.joint_name)

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
        card.setStyleSheet('QFrame { background: #1a2d3d; border-radius: 8px; }')
        layout = QVBoxLayout(card)
        layout.setSpacing(4)
        layout.setContentsMargins(12, 10, 12, 10)

        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(
            'color: #9cdcfe; font-size: 13px; font-weight: 600; background: transparent;'
        )
        layout.addWidget(title_lbl)

        desc_lbl = QLabel(desc)
        desc_lbl.setStyleSheet('color: #aaaaaa; font-size: 12px; background: transparent;')
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
            label.setStyleSheet('background: #1a1a1a; border-radius: 6px;')
        else:
            label.setText(fallback_text)
            label.setStyleSheet(
                'background: #2a2a2a; border-radius: 6px; color: #555; font-size: 12px; '
                'border: 1px dashed #444;'
            )
            label.setWordWrap(True)
        return label

    # ─── UI 설정 ──────────────────────────────────────────────────────────────

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(10)

        title = QLabel('Robot Calibration')
        title.setStyleSheet('color: #ffffff; font-size: 18px; font-weight: 600;')
        main_layout.addWidget(title)

        main_layout.addWidget(self._build_step_indicator())

        # 상태 메시지 (백엔드에서 오는 메시지 표시)
        self._status_bar = QLabel('')
        self._status_bar.setStyleSheet('color: #858585; font-size: 11px; padding: 2px 4px;')
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
            lbl.setFixedHeight(28)
            lbl.setContentsMargins(4, 0, 4, 0)
            self._step_labels.append(lbl)
            layout.addWidget(lbl, 2)

            if i < len(steps) - 1:
                sep = QLabel('›')
                sep.setAlignment(Qt.AlignmentFlag.AlignCenter)
                sep.setStyleSheet('color: #444; font-size: 16px;')
                sep.setFixedWidth(20)
                layout.addWidget(sep)

        return widget

    def _update_step_indicator(self):
        for i, lbl in enumerate(self._step_labels):
            if i < self._current_step:
                lbl.setStyleSheet(
                    'color: #4ec9b0; font-size: 12px; background: transparent; padding: 2px 4px;'
                )
            elif i == self._current_step:
                lbl.setStyleSheet(
                    'color: #ffffff; font-size: 12px; font-weight: 700; '
                    'background: #1a2d3d; border-radius: 4px; padding: 2px 4px;'
                )
            else:
                lbl.setStyleSheet(
                    'color: #555; font-size: 12px; background: transparent; padding: 2px 4px;'
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
        conn_group.setStyleSheet("""
            QGroupBox {
                color: #cccccc; font-size: 12px; font-weight: 600;
                border: 1px solid #3c3c3c; border-radius: 6px;
                margin-top: 6px; padding-top: 6px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
        """)
        conn_layout = QHBoxLayout(conn_group)
        conn_layout.setSpacing(8)

        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(160)
        conn_layout.addWidget(QLabel('Port:'))
        conn_layout.addWidget(self._port_combo)

        refresh_btn = QPushButton('↺')
        refresh_btn.setFixedSize(28, 28)
        refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #3c3c3c; border: 1px solid #555;
                border-radius: 3px; color: #cccccc; font-size: 14px;
            }
            QPushButton:hover { background-color: #505050; }
        """)
        refresh_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(refresh_btn)

        conn_layout.addWidget(QLabel('Arm:'))
        self._role_combo = QComboBox()
        self._role_combo.addItems(['follower', 'leader'])
        self._role_combo.setFixedWidth(100)
        conn_layout.addWidget(self._role_combo)

        self._connect_btn = QPushButton('Connect')
        self._connect_btn.setFixedHeight(32)
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
            self._make_image_label('explore.jpg', '📷  참고 사진 준비 중\n관절을 끝까지 돌리는 모습', height=120)
        )

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet('QScrollArea { border: none; background: transparent; }')

        sliders_widget = QWidget()
        sliders_layout = QVBoxLayout(sliders_widget)
        sliders_layout.setSpacing(4)
        sliders_layout.setContentsMargins(4, 4, 4, 4)

        for name in JOINT_NAMES:
            slider = JointRangeSlider(name)
            self._sliders[name] = slider
            sliders_layout.addWidget(slider)

        sliders_layout.addStretch()
        scroll.setWidget(sliders_widget)
        layout.addWidget(scroll, 1)

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
            dlg.setStyleSheet("""
                QMessageBox { background-color: #1e1e1e; color: #cccccc; }
                QLabel { color: #cccccc; font-size: 13px; }
                QPushButton {
                    background-color: #0e639c; color: white;
                    border: none; border-radius: 4px; padding: 6px 20px;
                    font-weight: 600; min-width: 80px;
                }
                QPushButton:hover { background-color: #1177bb; }
            """)
            dlg.exec()
            return
        if self._ui_node:
            self._ui_node.send_command('save')

    def _on_back_to_connect(self):
        if self._ui_node:
            self._ui_node.send_command('disconnect')
        # 'idle' 상태 수신 시 _on_status가 자동으로 step 0으로 이동

    def _on_back_to_center(self):
        self._go_to_step(1)

# ─── ROS2 콜백 ────────────────────────────────────────────────────────────

    def _on_status(self, status: str, message: str, data: dict):
        self._cal_status = status

        if status == 'error':
            self._status_bar.setStyleSheet(
                'color: #f48771; font-size: 11px; padding: 2px 4px;'
            )
            self._status_bar.setText(message)
            return

        self._status_bar.setStyleSheet('color: #858585; font-size: 11px; padding: 2px 4px;')
        self._status_bar.setText(message)

        if status == 'connected':
            self._connected = True
            self._go_to_step(1)
        elif status == 'idle':
            self._connected = False
            self._go_to_step(0)
        elif status == 'homing_set':
            # 슬라이더 초기화 (중앙값부터 추적 시작)
            for slider in self._sliders.values():
                slider.set_range(HALF_TURN, HALF_TURN, HALF_TURN)
            self._go_to_step(2)
        elif status == 'saved':
            self._status_bar.setStyleSheet('color: #4ec9b0; font-size: 11px; padding: 2px 4px;')
            file_path = data.get('file_path', '')
            dlg = QMessageBox(self)
            dlg.setWindowTitle('캘리브레이션 저장 완료')
            dlg.setText(f'캘리브레이션이 저장되었습니다\n{file_path}')
            dlg.setStandardButtons(QMessageBox.StandardButton.Ok)
            dlg.setStyleSheet("""
                QMessageBox { background-color: #1e1e1e; color: #cccccc; }
                QLabel { color: #cccccc; font-size: 13px; }
                QPushButton {
                    background-color: #0e639c; color: white;
                    border: none; border-radius: 4px; padding: 6px 20px;
                    font-weight: 600; min-width: 80px;
                }
                QPushButton:hover { background-color: #1177bb; }
            """)
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
