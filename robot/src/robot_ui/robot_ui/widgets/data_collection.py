import asyncio
from datetime import datetime
from functools import partial
from pathlib import Path

import cv2
import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QProgressBar,
    QDialog, QFrame, QGridLayout, QScrollArea,
)
from PySide6.QtCore import Qt, Signal, QObject
from PySide6.QtGui import QImage, QPixmap, QPainter, QFont, QColor
from rclpy.logging import get_logger
from sensor_msgs.msg import Image, JointState

from ..utils import ApiClient
from ..utils.joint_state_collector import JointStateCollector
from ..services import UploadService, MultiCameraRecordingService, ParquetWriter, MetadataService
from .theme import (
    GLASS_BG, GLASS_BORDER, RADIUS_LG, RADIUS_MD,
    TEXT_H1, TEXT_BODY, TEXT_MUTED, TEXT_DISABLED,
    ACCENT, ACCENT_GREEN, ACCENT_GREEN_END, ACCENT_RED,
    btn_danger, btn_success, progressbar_style,
)

logger = get_logger('DataCollection')

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
_POS_MIN, _POS_MAX = 0, 4095


# ─── Qt 스레드 안전 시그널 ─────────────────────────────────────────────────────

class _FrameSignal(QObject):
    received = Signal(str, object)


class _JointSignal(QObject):
    received = Signal(list)


# ─── 카메라 표시 위젯 ──────────────────────────────────────────────────────────

class _CameraWidget(QFrame):
    def __init__(self, role: str, parent=None):
        super().__init__(parent)
        self.role = role
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: 1px solid {GLASS_BORDER};
                border-radius: {RADIUS_MD};
            }}
        """)
        self.setMinimumSize(280, 210)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        role_label = QLabel(role)
        role_label.setStyleSheet(f"color: {ACCENT}; font-size: 11px; font-weight: 600; background: transparent; border: none;")
        layout.addWidget(role_label)

        self._image_label = QLabel()
        self._image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._image_label.setStyleSheet(
            "background-color: rgba(196,181,253,0.15); border-radius: 6px;"
            f" color: {TEXT_DISABLED}; border: 1px solid rgba(196,181,253,0.25);"
        )
        self._image_label.setText('대기 중...')
        layout.addWidget(self._image_label, 1)

    def update_image(self, cv_image: np.ndarray):
        size = self._image_label.size()
        h, w = cv_image.shape[:2]
        scale = min(size.width() / max(w, 1), size.height() / max(h, 1))
        if scale > 0:
            nw, nh = int(w * scale), int(h * scale)
            img = cv2.resize(cv_image, (nw, nh))
        else:
            img = cv_image
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        rh, rw, ch = rgb.shape
        q = QImage(rgb.data, rw, rh, ch * rw, QImage.Format.Format_RGB888)
        self._image_label.setPixmap(QPixmap.fromImage(q))


# ─── 관절 상태 행 (팔로워만) ───────────────────────────────────────────────────

class _JointRow(QWidget):
    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 1, 0, 1)
        layout.setSpacing(6)

        lbl = QLabel(name)
        lbl.setFixedWidth(96)
        lbl.setStyleSheet(f'color: {TEXT_BODY}; font-size: 11px; background: transparent;')
        layout.addWidget(lbl)

        self._bar = QProgressBar()
        self._bar.setRange(_POS_MIN, _POS_MAX)
        self._bar.setValue(2048)
        self._bar.setFixedHeight(12)
        self._bar.setTextVisible(False)
        self._bar.setStyleSheet(progressbar_style(ACCENT_GREEN, ACCENT_GREEN_END))
        layout.addWidget(self._bar, 1)

        self._val = QLabel('2048')
        self._val.setFixedWidth(36)
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._val.setStyleSheet(f'color: {ACCENT_GREEN}; font-size: 10px; font-family: monospace; background: transparent;')
        layout.addWidget(self._val)

    def update_value(self, v: float):
        iv = int(max(_POS_MIN, min(_POS_MAX, v)))
        self._bar.setValue(iv)
        self._val.setText(str(iv))


# ─── 인코딩/업로드 진행 팝업 ──────────────────────────────────────────────────

class _PostProcessDialog(QDialog):
    """수집 완료 후 인코딩·업로드가 끝날 때까지 사용자 조작을 막는 모달 팝업"""

    def __init__(self, total: int, already_done: int, parent=None):
        super().__init__(parent)
        self._total = total
        self.setWindowTitle("처리 중...")
        self.setWindowFlags(
            Qt.WindowType.Dialog |
            Qt.WindowType.CustomizeWindowHint |
            Qt.WindowType.WindowTitleHint
        )
        self.setWindowModality(Qt.WindowModality.ApplicationModal)
        self.setFixedSize(400, 140)
        self.setStyleSheet(f"background-color: white; color: {TEXT_H1};")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)

        self._status_label = QLabel("인코딩 및 업로드 중...")
        self._status_label.setStyleSheet(f"color: {TEXT_BODY}; font-size: 13px; background: transparent;")
        layout.addWidget(self._status_label)

        self._bar = QProgressBar()
        self._bar.setRange(0, total)
        self._bar.setValue(already_done)
        self._bar.setFixedHeight(12)
        self._bar.setTextVisible(False)
        self._bar.setStyleSheet(progressbar_style(ACCENT, '#a78bfa'))
        layout.addWidget(self._bar)

        self._progress_label = QLabel(f"에피소드 {already_done} / {total} 완료")
        self._progress_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px; background: transparent;")
        self._progress_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self._progress_label)

    def set_status(self, text: str):
        self._status_label.setText(text)

    def set_progress(self, done: int):
        self._bar.setValue(done)
        self._progress_label.setText(f"에피소드 {done} / {self._total} 완료")
        if done >= self._total:
            self.accept()

    def closeEvent(self, event):
        event.ignore()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            return
        super().keyPressEvent(event)


# ─── 3-2-1 카운트다운 오버레이 ─────────────────────────────────────────────────

class _CountdownOverlay(QWidget):
    """Record 버튼 클릭 후 3-2-1 카운트다운을 화면 중앙에 표시하는 오버레이"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._number = 3
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.setVisible(False)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        painter.fillRect(self.rect(), QColor(0, 0, 0, 140))

        cx, cy = self.width() // 2, self.height() // 2
        r = 90
        painter.setBrush(QColor(220, 38, 38, 230))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        font = QFont()
        font.setPointSize(72)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, str(self._number))

        painter.end()

    async def run(self):
        if parent := self.parent():
            self.setGeometry(parent.rect())
        self.raise_()
        self.show()
        for n in (3, 2, 1):
            self._number = n
            self.update()
            await asyncio.sleep(1.0)
        self.hide()


# ─── 메인 패널 ────────────────────────────────────────────────────────────────

class DataCollectionPanel(QWidget):
    """데이터 수집 패널 — 카메라 뷰, 관절 상태, 에피소드/시간 카운트다운 포함"""

    recording_finished = Signal()

    _PHASE_STYLE = {
        'recording': ('● 수집 중',  ACCENT_RED,    ACCENT_RED),
        'waiting':   ('● 대기 중',  '#ca8a04',     '#ca8a04'),
        'idle':      ('● 대기',     TEXT_DISABLED, '#d1d5db'),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.settings: dict = {}
        self.is_recording = False
        self.ros_node = None
        self.joint_collector = None
        self.recording_service = None
        self.session_dir: Path = None
        self.upload_service = UploadService(ApiClient())
        self._frame_subscriptions: list = []
        self.recording_task = None

        self._frame_signal = _FrameSignal()
        self._frame_signal.received.connect(self._on_frame)
        self._joint_signal = _JointSignal()
        self._joint_signal.received.connect(self._on_joints)
        self._joint_sub = None

        self._camera_widgets: dict[str, _CameraWidget] = {}
        self._joint_rows: list[_JointRow] = []
        self._post_dialog: _PostProcessDialog | None = None

        self._setup_ui()
        self._countdown_overlay = _CountdownOverlay(self)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._countdown_overlay.setGeometry(self.rect())

    # ------------------------------------------------------------------
    # UI 구성
    # ------------------------------------------------------------------

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(10)

        # ── 제목 + 상태 배지 ──────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel('Data Collection')
        title.setStyleSheet(f"color: {TEXT_H1}; font-size: 22px; font-weight: 700; background: transparent;")
        header.addWidget(title)
        header.addStretch()
        self._phase_label = QLabel('● 대기')
        self._phase_label.setStyleSheet(f"color: {TEXT_DISABLED}; font-size: 13px; font-weight: 600; background: transparent;")
        header.addWidget(self._phase_label)
        root.addLayout(header)

        # ── 에피소드 + 남은 시간 한 줄 ───────────────────────────────
        info_row = QHBoxLayout()
        info_row.setSpacing(20)
        self._episode_label = QLabel('에피소드 —')
        self._episode_label.setStyleSheet(f"color: {TEXT_BODY}; font-size: 12px; background: transparent;")
        info_row.addWidget(self._episode_label)
        self._time_label = QLabel('남은 시간 —')
        self._time_label.setStyleSheet(f"color: {TEXT_BODY}; font-size: 12px; background: transparent;")
        info_row.addWidget(self._time_label)
        info_row.addStretch()
        self._status_label = QLabel('Ready to record')
        self._status_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px; background: transparent;")
        info_row.addWidget(self._status_label)
        root.addLayout(info_row)

        # ── 페이즈 카운트다운 바 ──────────────────────────────────────
        self._phase_bar = QProgressBar()
        self._phase_bar.setFixedHeight(8)
        self._phase_bar.setTextVisible(False)
        self._phase_bar.setRange(0, 1000)
        self._phase_bar.setValue(0)
        self._phase_bar.setStyleSheet(progressbar_style('#d1d5db'))

        # ── 에피소드 전체 진행 바 ─────────────────────────────────────
        ep_row = QHBoxLayout()
        ep_lbl = QLabel('전체 진행')
        ep_lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px; background: transparent;")
        ep_row.addWidget(ep_lbl)
        self._progress_bar = QProgressBar()
        self._progress_bar.setFixedHeight(8)
        self._progress_bar.setTextVisible(False)
        self._progress_bar.setRange(0, 1)
        self._progress_bar.setValue(0)
        self._progress_bar.setStyleSheet(progressbar_style(ACCENT, '#a78bfa'))
        ep_row.addWidget(self._progress_bar, 1)
        self._progress_label = QLabel('0 / 0')
        self._progress_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px; background: transparent;")
        ep_row.addWidget(self._progress_label)

        # ── 진행 글래스 카드 ─────────────────────────────────────────
        progress_card = QFrame()
        progress_card.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: 1px solid {GLASS_BORDER};
                border-radius: {RADIUS_LG};
            }}
        """)
        progress_inner = QVBoxLayout(progress_card)
        progress_inner.setContentsMargins(16, 12, 16, 12)
        progress_inner.setSpacing(8)
        progress_inner.addWidget(self._phase_bar)
        progress_inner.addLayout(ep_row)
        root.addWidget(progress_card)

        # ── 메인 콘텐츠 (카메라 + 관절) ──────────────────────────────
        content = QHBoxLayout()
        content.setSpacing(12)

        cam_frame = QFrame()
        cam_frame.setStyleSheet("QFrame { background-color: transparent; border: none; }")
        self._cam_layout = QGridLayout(cam_frame)
        self._cam_layout.setContentsMargins(0, 0, 0, 0)
        self._cam_layout.setSpacing(8)
        content.addWidget(cam_frame, 3)

        joint_card = QFrame()
        joint_card.setStyleSheet(f"""
            QFrame {{
                background-color: {GLASS_BG};
                border: 1px solid {GLASS_BORDER};
                border-radius: {RADIUS_LG};
            }}
        """)
        joint_layout = QVBoxLayout(joint_card)
        joint_layout.setContentsMargins(12, 14, 12, 12)
        joint_layout.setSpacing(2)
        section_lbl = QLabel('팔로워암 관절')
        section_lbl.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 9px; font-weight: 700;"
            " letter-spacing: 1px; text-transform: uppercase; background: transparent;"
        )
        joint_layout.addWidget(section_lbl)
        for name in JOINT_NAMES:
            row = _JointRow(name)
            self._joint_rows.append(row)
            joint_layout.addWidget(row)
        joint_layout.addStretch()
        content.addWidget(joint_card, 1)

        root.addLayout(content, 1)

        # ── Record 버튼 ───────────────────────────────────────────────
        self.record_btn = QPushButton('Record')
        self.record_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.record_btn.setFixedHeight(48)
        self.record_btn.setStyleSheet(btn_danger())
        self.record_btn.clicked.connect(self._on_record_clicked)
        root.addWidget(self.record_btn)

        from PySide6.QtCore import QPropertyAnimation, QEasingCurve
        from PySide6.QtWidgets import QGraphicsOpacityEffect
        self._pulse_effect = QGraphicsOpacityEffect(self.record_btn)
        self._pulse_effect.setOpacity(1.0)
        self.record_btn.setGraphicsEffect(self._pulse_effect)
        self._pulse_anim = QPropertyAnimation(self._pulse_effect, b'opacity')
        self._pulse_anim.setDuration(700)
        self._pulse_anim.setStartValue(0.65)
        self._pulse_anim.setEndValue(1.0)
        self._pulse_anim.setEasingCurve(QEasingCurve.Type.InOutSine)
        self._pulse_anim.setLoopCount(-1)

    # ------------------------------------------------------------------
    # 카메라 위젯 동적 생성
    # ------------------------------------------------------------------

    def _rebuild_camera_grid(self, camera_roles: dict):
        for w in self._camera_widgets.values():
            self._cam_layout.removeWidget(w)
            w.deleteLater()
        self._camera_widgets.clear()

        cols = 2
        for idx, role in enumerate(camera_roles):
            w = _CameraWidget(role)
            self._camera_widgets[role] = w
            self._cam_layout.addWidget(w, idx // cols, idx % cols)

    # ------------------------------------------------------------------
    # 공개 API
    # ------------------------------------------------------------------

    def set_ros_node(self, node):
        self.ros_node = node
        self.joint_collector = JointStateCollector(node)
        self._joint_sub = node.create_subscription(
            JointState, '/follower/joint_states',
            lambda msg: self._joint_signal.received.emit(list(msg.position)), 10
        )

    def update_camera_roles(self, camera_roles: dict):
        if self.is_recording:
            return
        self.settings['camera_roles'] = camera_roles
        self._rebuild_camera_grid(camera_roles)

    def set_recording_config(self, settings: dict):
        self.settings = settings

        session_id  = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = Path(f"/tmp/dataset_{session_id}")

        metadata_service = MetadataService()
        global_offset = metadata_service.load_or_init(
            session_dir / "meta",
            camera_roles=settings['camera_roles'],
            fps=30,
        )
        parquet_writer = ParquetWriter(global_frame_offset=global_offset)

        self.recording_service = MultiCameraRecordingService(
            upload_service=self.upload_service,
            joint_collector=self.joint_collector,
            metadata_service=metadata_service,
            parquet_writer=parquet_writer,
        )
        self.recording_service.set_display_callback(
            lambda role, img: self._frame_signal.received.emit(role, img)
        )
        self.session_dir = session_dir

        episodes = settings.get('episodes', 1)
        self._progress_bar.setMaximum(episodes)
        self._progress_bar.setValue(0)
        self._progress_label.setText(f"0 / {episodes}")

        self._rebuild_camera_grid(settings['camera_roles'])
        self._status_label.setText(
            f"준비: {episodes}개 에피소드 × {settings['data_length']}s"
        )

    # ------------------------------------------------------------------
    # Qt 슬롯 — 스레드 안전
    # ------------------------------------------------------------------

    def _on_frame(self, role: str, img: np.ndarray):
        if role in self._camera_widgets:
            self._camera_widgets[role].update_image(img)

    def _on_joints(self, positions: list):
        for i, row in enumerate(self._joint_rows):
            if i < len(positions):
                row.update_value(positions[i])

    # ------------------------------------------------------------------
    # 수집 흐름
    # ------------------------------------------------------------------

    def _on_record_clicked(self):
        if not self.is_recording:
            self.recording_task = asyncio.create_task(self._start_recording())

    async def _start_recording(self):
        if not self.ros_node:
            logger.error("ROS2 node not available")
            return
        if not self.recording_service:
            logger.error("Recording service not configured. Call set_recording_config() first.")
            return

        episodes     = self.settings.get('episodes', 1)
        camera_roles = self.settings.get('camera_roles', {})

        self.is_recording = True
        self._pulse_anim.start()
        self.record_btn.setEnabled(False)
        self._progress_bar.setMaximum(episodes)
        self._progress_bar.setValue(0)
        self._progress_label.setText(f"0 / {episodes}")

        await self._countdown_overlay.run()

        self._frame_subscriptions = []
        for role, topic in camera_roles.items():
            sub = self.ros_node.create_subscription(
                Image, topic,
                partial(self.recording_service.on_frame_received, role), 10
            )
            self._frame_subscriptions.append(sub)

        await self.recording_service.run(
            settings=self.settings,
            session_dir=self.session_dir,
            on_status=self._on_status,
            on_progress=self._on_progress,
            ask_result=self._ask_episode_result,
            on_countdown=self._on_countdown,
            on_collection_done=self._on_collection_done,
        )

        for sub in self._frame_subscriptions:
            self.ros_node.destroy_subscription(sub)
        self._frame_subscriptions = []

        self._progress_bar.setValue(episodes)
        self._progress_label.setText(f"{episodes} / {episodes}")
        self._update_phase('idle')
        self._time_label.setText('완료')
        self._status_label.setText('수집 완료!')
        self.is_recording = False
        self._pulse_anim.stop()
        self._pulse_effect.setOpacity(1.0)
        self.record_btn.setEnabled(True)
        self.recording_finished.emit()

    def _on_status(self, text: str):
        self._status_label.setText(text)
        if self._post_dialog and self._post_dialog.isVisible():
            self._post_dialog.set_status(text)

    def _on_progress(self, episode_idx: int):
        done = episode_idx + 1
        total = self.settings.get('episodes', 1)
        self._progress_bar.setValue(done)
        self._progress_label.setText(f"{done} / {total}")
        if self._post_dialog and self._post_dialog.isVisible():
            self._post_dialog.set_progress(done)

    def _on_collection_done(self, total: int):
        already_done = self._progress_bar.value()
        if already_done < total:
            self._post_dialog = _PostProcessDialog(total, already_done, self)
            self._post_dialog.show()

    def _on_countdown(self, phase: str, remaining: float, total: float, ep_idx: int, total_eps: int):
        self._update_phase(phase)
        pct = int((1.0 - remaining / max(total, 0.001)) * 1000)
        self._phase_bar.setValue(pct)
        self._time_label.setText(f"남은 시간  {remaining:.1f}s")
        self._episode_label.setText(f"에피소드  {ep_idx + 1} / {total_eps}")

    def _update_phase(self, phase: str):
        text, dot_color, bar_color = self._PHASE_STYLE.get(phase, self._PHASE_STYLE['idle'])
        self._phase_label.setText(text)
        self._phase_label.setStyleSheet(
            f"color: {dot_color}; font-size: 13px; font-weight: 600; background: transparent;"
        )
        bar_end = '#a78bfa' if bar_color == ACCENT else bar_color
        self._phase_bar.setStyleSheet(progressbar_style(bar_color, bar_end))

    # ------------------------------------------------------------------
    # 에피소드 결과 팝업
    # ------------------------------------------------------------------

    async def _ask_episode_result(self, episode_index: int) -> bool:
        loop   = asyncio.get_event_loop()
        future = loop.create_future()

        dialog = QDialog(self)
        dialog.setWindowTitle(f"에피소드 {episode_index + 1} 결과")
        dialog.setStyleSheet(f"background-color: white; color: {TEXT_H1};")
        dialog.setFixedSize(320, 130)

        outer = QVBoxLayout(dialog)
        outer.setContentsMargins(24, 20, 24, 20)
        outer.setSpacing(14)

        label = QLabel(f"에피소드 {episode_index + 1} — 성공했나요?")
        label.setStyleSheet(f"color: {TEXT_H1}; font-size: 14px; background: transparent;")
        outer.addWidget(label)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        success_btn = QPushButton("✓ 성공")
        success_btn.setStyleSheet(btn_success())
        fail_btn = QPushButton("✗ 실패")
        fail_btn.setStyleSheet(btn_danger())

        def _resolve(result: bool):
            if not future.done():
                future.set_result(result)
            dialog.accept()

        success_btn.clicked.connect(lambda: _resolve(True))
        fail_btn.clicked.connect(lambda: _resolve(False))

        btn_layout.addWidget(success_btn)
        btn_layout.addWidget(fail_btn)
        outer.addLayout(btn_layout)
        dialog.show()

        return await asyncio.wrap_future(future)
