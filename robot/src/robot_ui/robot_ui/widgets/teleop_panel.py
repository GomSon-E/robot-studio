import threading

import serial.tools.list_ports
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFrame, QComboBox, QProgressBar, QScrollArea,
)
from PySide6.QtCore import Qt, Signal, QObject

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from robot_driver.leader_arm_node import LeaderArmNode
from robot_driver.follower_arm_node import FollowerArmNode
from robot_driver.teleop_node import TeleopNode

from .theme import (
    GLASS_BG, GLASS_BORDER, RADIUS_LG,
    TEXT_H1, TEXT_BODY, TEXT_MUTED, TEXT_DISABLED,
    ACCENT, ACCENT_GREEN, ACCENT_RED,
    btn_primary, btn_success, btn_danger, btn_warning, btn_icon_sm,
    combobox_style, scrollbar_style, progressbar_style,
)

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
POS_MIN = 0
POS_MAX = 4095

_LED_STYLE = {
    'connected':    f'color: {ACCENT_GREEN};',
    'disconnected': f'color: {ACCENT_RED};',
    'connecting':   'color: #ca8a04;',
    'idle':         f'color: {TEXT_DISABLED};',
}


# ─── ROS2 → Qt 시그널 ────────────────────────────────────────────────────────

class TeleopSignals(QObject):
    leader_joints_received   = Signal(list)
    follower_joints_received = Signal(list)
    leader_status_changed    = Signal(str)
    follower_status_changed  = Signal(str)
    teleop_status_changed    = Signal(str)


# ─── UI 전용 ROS2 노드 ─────────────────────────────────────────────────────────

class TeleopUINode(Node):
    def __init__(self, signals: TeleopSignals):
        super().__init__('teleop_ui_node')
        self._signals = signals

        self.create_subscription(JointState, '/leader/joint_states',        self._on_leader_joints,   10)
        self.create_subscription(JointState, '/follower/joint_states',      self._on_follower_joints, 10)
        self.create_subscription(String,     '/leader/connection_status',   self._on_leader_status,   10)
        self.create_subscription(String,     '/follower/connection_status', self._on_follower_status, 10)
        self.create_subscription(String,     '/teleop/status',              self._on_teleop_status,   10)

        self._teleop_pub = self.create_publisher(Bool, '/teleop/command', 10)

    def _on_leader_joints(self, msg: JointState):
        self._signals.leader_joints_received.emit(list(msg.position))

    def _on_follower_joints(self, msg: JointState):
        self._signals.follower_joints_received.emit(list(msg.position))

    def _on_leader_status(self, msg: String):
        self._signals.leader_status_changed.emit(msg.data)

    def _on_follower_status(self, msg: String):
        self._signals.follower_status_changed.emit(msg.data)

    def _on_teleop_status(self, msg: String):
        self._signals.teleop_status_changed.emit(msg.data)

    def send_teleop_command(self, active: bool):
        self._teleop_pub.publish(Bool(data=active))


# ─── 암 연결 위젯 ─────────────────────────────────────────────────────────────

class ArmConnectionWidget(QFrame):
    connect_clicked = Signal(str)

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setStyleSheet("QFrame { background-color: transparent; border: none; }")
        self._setup_ui(title)
        self._refresh_ports()

    def _setup_ui(self, title: str):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(8)

        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(
            f"color: {TEXT_BODY}; font-size: 13px; font-weight: 600;"
            " background: transparent;"
        )
        outer.addWidget(title_lbl)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)

        self._led = QLabel('●')
        self._led.setStyleSheet(_LED_STYLE['idle'])
        self._led.setFixedWidth(14)
        row.addWidget(self._led)

        self._status_label = QLabel('대기')
        self._status_label.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 13px; background: transparent;')
        self._status_label.setFixedWidth(80)
        row.addWidget(self._status_label)

        self._combo = QComboBox()
        self._combo.setStyleSheet(combobox_style())
        row.addWidget(self._combo, 1)

        refresh_btn = QPushButton('↺')
        refresh_btn.setFixedSize(30, 30)
        refresh_btn.setToolTip('포트 목록 새로고침')
        refresh_btn.setStyleSheet(btn_icon_sm())
        refresh_btn.clicked.connect(self._refresh_ports)
        row.addWidget(refresh_btn)

        self._connect_btn = QPushButton('연결')
        self._connect_btn.setFixedHeight(30)
        self._connect_btn.setStyleSheet(btn_primary())
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        row.addWidget(self._connect_btn)

        outer.addLayout(row)

    def _refresh_ports(self):
        self._combo.clear()
        usb_ports = [p for p in serial.tools.list_ports.comports() if '/dev/ttyS' not in p.device]
        for p in usb_ports:
            self._combo.addItem(f'{p.device} — {p.description}', p.device)
        if self._combo.count() == 0:
            self._combo.addItem('포트 없음', '')

    def _on_connect_clicked(self):
        port = self._combo.currentData()
        if not port:
            return
        self.set_connecting()
        self.connect_clicked.emit(port)

    def set_connecting(self):
        self._led.setStyleSheet(_LED_STYLE['connecting'])
        self._status_label.setText('연결 중...')
        self._connect_btn.setEnabled(False)
        self._combo.setEnabled(False)

    def set_status(self, status: str):
        if status == 'connected':
            self._led.setStyleSheet(_LED_STYLE['connected'])
            self._status_label.setText('연결됨')
            self._connect_btn.setEnabled(False)
            self._combo.setEnabled(False)
        else:
            self._led.setStyleSheet(_LED_STYLE['disconnected'])
            self._status_label.setText('연결 끊김')
            self._connect_btn.setEnabled(True)
            self._combo.setEnabled(True)


# ─── 관절 상태 행 ──────────────────────────────────────────────────────────────

class JointStateRow(QWidget):
    def __init__(self, joint_name: str, parent=None):
        super().__init__(parent)
        self._setup_ui(joint_name)

    def _setup_ui(self, joint_name: str):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(8)

        name_label = QLabel(joint_name)
        name_label.setFixedWidth(110)
        name_label.setStyleSheet(f'color: {TEXT_BODY}; font-size: 13px; background: transparent;')
        layout.addWidget(name_label)

        self._leader_bar = QProgressBar()
        self._leader_bar.setRange(POS_MIN, POS_MAX)
        self._leader_bar.setValue(2048)
        self._leader_bar.setFixedHeight(14)
        self._leader_bar.setTextVisible(False)
        self._leader_bar.setStyleSheet(progressbar_style(ACCENT, '#a78bfa'))
        layout.addWidget(self._leader_bar, 1)

        self._leader_val = QLabel('2048')
        self._leader_val.setFixedWidth(38)
        self._leader_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._leader_val.setStyleSheet(f'color: {ACCENT}; font-size: 12px; font-family: monospace; background: transparent;')
        layout.addWidget(self._leader_val)

        sep = QLabel('→')
        sep.setStyleSheet(f'color: {TEXT_DISABLED}; font-size: 12px; background: transparent;')
        layout.addWidget(sep)

        self._follower_bar = QProgressBar()
        self._follower_bar.setRange(POS_MIN, POS_MAX)
        self._follower_bar.setValue(2048)
        self._follower_bar.setFixedHeight(14)
        self._follower_bar.setTextVisible(False)
        self._follower_bar.setStyleSheet(progressbar_style(ACCENT_GREEN, '#34d399'))
        layout.addWidget(self._follower_bar, 1)

        self._follower_val = QLabel('2048')
        self._follower_val.setFixedWidth(38)
        self._follower_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._follower_val.setStyleSheet(f'color: {ACCENT_GREEN}; font-size: 12px; font-family: monospace; background: transparent;')
        layout.addWidget(self._follower_val)

    def update_leader(self, value: float):
        v = int(max(POS_MIN, min(POS_MAX, value)))
        self._leader_bar.setValue(v)
        self._leader_val.setText(str(v))

    def update_follower(self, value: float):
        v = int(max(POS_MIN, min(POS_MAX, value)))
        self._follower_bar.setValue(v)
        self._follower_val.setText(str(v))


# ─── 메인 패널 ────────────────────────────────────────────────────────────────

class TeleopPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._ui_node: TeleopUINode | None = None
        self._teleop_node: TeleopNode | None = None
        self._leader_node: LeaderArmNode | None = None
        self._follower_node: FollowerArmNode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._ros_thread: threading.Thread | None = None
        self._running = False
        self._teleop_active = False
        self._leader_connected = False
        self._follower_connected = False

        self._signals = TeleopSignals()
        self._signals.leader_joints_received.connect(self._on_leader_joints)
        self._signals.follower_joints_received.connect(self._on_follower_joints)
        self._signals.leader_status_changed.connect(self._on_leader_status)
        self._signals.follower_status_changed.connect(self._on_follower_status)
        self._signals.teleop_status_changed.connect(self._on_teleop_status)

        self._setup_ui()
        self._init_ros()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(16)

        title = QLabel('Teleop Control')
        title.setStyleSheet(f'color: {TEXT_H1}; font-size: 22px; font-weight: 700; background: transparent;')
        main_layout.addWidget(title)

        subtitle = QLabel('리더 → 팔로워 암  (SO-ARM 101)')
        subtitle.setStyleSheet(f'color: {TEXT_MUTED}; font-size: 13px; background: transparent;')
        main_layout.addWidget(subtitle)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(scrollbar_style())

        content = QWidget()
        content.setStyleSheet('background-color: transparent;')
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 8, 0)
        content_layout.setSpacing(12)

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
            f'color: {TEXT_MUTED}; font-size: 13px; font-weight: 600; '
            f'background: transparent;'
        )
        conn_card_layout.addWidget(conn_label)
        conn_card_layout.addSpacing(8)

        self._leader_widget = ArmConnectionWidget('리더암')
        self._leader_widget.connect_clicked.connect(self._on_leader_connect)
        conn_card_layout.addWidget(self._leader_widget)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFixedHeight(1)
        sep.setStyleSheet("background-color: rgba(196,181,253,0.3); border: none;")
        conn_card_layout.addWidget(sep)

        self._follower_widget = ArmConnectionWidget('팔로워암')
        self._follower_widget.connect_clicked.connect(self._on_follower_connect)
        conn_card_layout.addWidget(self._follower_widget)

        content_layout.addWidget(conn_card)

        joint_card = QFrame()
        joint_card.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: none;
                border-radius: {RADIUS_LG};
            }}
        """)
        joint_layout = QVBoxLayout(joint_card)
        joint_layout.setContentsMargins(14, 14, 14, 12)
        joint_layout.setSpacing(4)

        section_lbl = QLabel('관절 상태')
        section_lbl.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 9px; font-weight: 700;"
            " letter-spacing: 1px; background: transparent;"
        )
        joint_layout.addWidget(section_lbl)

        legend_layout = QHBoxLayout()
        legend_layout.addSpacing(118)
        leader_legend = QLabel('● 리더')
        leader_legend.setStyleSheet(f'color: {ACCENT}; font-size: 12px; background: transparent;')
        legend_layout.addWidget(leader_legend)
        legend_layout.addStretch()
        follower_legend = QLabel('● 팔로워')
        follower_legend.setStyleSheet(f'color: {ACCENT_GREEN}; font-size: 12px; background: transparent;')
        legend_layout.addWidget(follower_legend)
        joint_layout.addLayout(legend_layout)

        self._joint_rows: list[JointStateRow] = []
        for name in JOINT_NAMES:
            row = JointStateRow(name)
            self._joint_rows.append(row)
            joint_layout.addWidget(row)

        content_layout.addWidget(joint_card)
        content_layout.addStretch()
        scroll.setWidget(content)
        main_layout.addWidget(scroll, 1)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        self._teleop_btn = QPushButton('텔레옵 시작')
        self._teleop_btn.setFixedHeight(44)
        self._teleop_btn.setEnabled(False)
        self._apply_teleop_btn_style(active=False)
        self._teleop_btn.clicked.connect(self._on_teleop_toggle)
        btn_layout.addWidget(self._teleop_btn, 1)

        estop_btn = QPushButton('긴급정지')
        estop_btn.setFixedHeight(44)
        estop_btn.setStyleSheet(btn_danger())
        estop_btn.clicked.connect(self._on_estop)
        btn_layout.addWidget(estop_btn, 1)

        main_layout.addLayout(btn_layout)

    def _apply_teleop_btn_style(self, active: bool):
        if active:
            self._teleop_btn.setStyleSheet(btn_warning())
        else:
            self._teleop_btn.setStyleSheet(btn_success())

    # ── ROS2 초기화 ──────────────────────────────────────────────────────────

    def _init_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self._executor = MultiThreadedExecutor()

            self._ui_node = TeleopUINode(self._signals)
            self._teleop_node = TeleopNode()
            self._executor.add_node(self._ui_node)
            self._executor.add_node(self._teleop_node)

            self._running = True
            self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self._ros_thread.start()
        except Exception:
            pass

    def _ros_spin(self):
        while self._running and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    # ── [연결] 버튼 핸들러 ────────────────────────────────────────────────────

    def _on_leader_connect(self, port: str):
        if not self._executor:
            return
        if self._leader_node is not None:
            self._executor.remove_node(self._leader_node)
            self._leader_node.destroy_node()
        self._leader_node = LeaderArmNode(
            parameter_overrides=[Parameter('port', Parameter.Type.STRING, port)]
        )
        self._executor.add_node(self._leader_node)

    def _on_follower_connect(self, port: str):
        if not self._executor:
            return
        if self._follower_node is not None:
            self._executor.remove_node(self._follower_node)
            self._follower_node.destroy_node()
        self._follower_node = FollowerArmNode(
            parameter_overrides=[Parameter('port', Parameter.Type.STRING, port)]
        )
        self._executor.add_node(self._follower_node)

    # ── ROS2 상태 수신 ───────────────────────────────────────────────────────

    def _on_leader_joints(self, positions: list):
        for i, row in enumerate(self._joint_rows):
            if i < len(positions):
                row.update_leader(positions[i])

    def _on_follower_joints(self, positions: list):
        for i, row in enumerate(self._joint_rows):
            if i < len(positions):
                row.update_follower(positions[i])

    def _on_leader_status(self, status: str):
        self._leader_widget.set_status(status)
        self._leader_connected = (status == 'connected')
        self._update_teleop_btn()

    def _on_follower_status(self, status: str):
        self._follower_widget.set_status(status)
        self._follower_connected = (status == 'connected')
        self._update_teleop_btn()

    def _on_teleop_status(self, status: str):
        self._teleop_active = (status == 'active')
        self._teleop_btn.setText('텔레옵 정지' if self._teleop_active else '텔레옵 시작')
        self._apply_teleop_btn_style(active=self._teleop_active)

    def _update_teleop_btn(self):
        self._teleop_btn.setEnabled(self._leader_connected and self._follower_connected)

    def _on_teleop_toggle(self):
        if self._ui_node:
            self._ui_node.send_teleop_command(not self._teleop_active)

    def _on_estop(self):
        if self._ui_node:
            self._ui_node.send_teleop_command(False)

    # ── 정리 ─────────────────────────────────────────────────────────────────

    def cleanup(self):
        self._running = False
        if self._ros_thread and self._ros_thread.is_alive():
            self._ros_thread.join(timeout=1.0)
        if self._executor:
            self._executor.shutdown()
        for node in [self._leader_node, self._follower_node, self._teleop_node, self._ui_node]:
            if node is not None:
                node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
