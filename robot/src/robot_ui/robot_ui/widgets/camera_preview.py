from typing import Optional
import threading

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout,
    QFrame, QScrollArea
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap

import rclpy
from rclpy.executors import MultiThreadedExecutor
import cv2
import numpy as np

from ..utils import ImageSignal, CameraSubscriberNode


class CameraPreviewWidget(QFrame):
    """개별 카메라 프리뷰 위젯"""

    def __init__(self, topic_name: str, parent=None):
        super().__init__(parent)
        self.topic_name = topic_name
        self.setStyleSheet("""
            QFrame {
                background-color: #2d2d2d;
                border-radius: 8px;
            }
        """)
        self.setMinimumSize(400, 300)

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        # 토픽 이름 라벨
        self.name_label = QLabel(self.topic_name)
        self.name_label.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-size: 12px;
                font-weight: 600;
                background-color: transparent;
            }
        """)
        layout.addWidget(self.name_label)

        # 이미지 표시 라벨
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("""
            QLabel {
                background-color: #1e1e1e;
                border-radius: 4px;
            }
        """)
        self.image_label.setText('Waiting for image...')
        layout.addWidget(self.image_label, 1)

    def update_image(self, cv_image: np.ndarray):
        """OpenCV 이미지를 QLabel에 표시"""
        label_size = self.image_label.size()
        h, w = cv_image.shape[:2]

        # 라벨 크기에 맞게 스케일 계산
        scale = min(label_size.width() / w, label_size.height() / h)
        if scale > 0:
            new_w, new_h = int(w * scale), int(h * scale)
            resized = cv2.resize(cv_image, (new_w, new_h))
        else:
            resized = cv_image

        # BGR to RGB
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        # numpy array to QImage
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        pixmap = QPixmap.fromImage(q_image)
        self.image_label.setPixmap(pixmap)


class CameraPreviewArea(QWidget):
    """메인 영역의 카메라 프리뷰 그리드"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.preview_widgets: dict[str, CameraPreviewWidget] = {}
        self.ros_node: Optional[CameraSubscriberNode] = None
        self.ros_thread: Optional[threading.Thread] = None
        self.executor: Optional[MultiThreadedExecutor] = None
        self.image_signal = ImageSignal()
        self.image_signal.image_received.connect(self._on_image_received)
        self._running = False

        self._setup_ui()
        self._init_ros()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # 헤더
        header_layout = QHBoxLayout()

        title = QLabel('Camera Preview')
        title.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-size: 18px;
                font-weight: 600;
            }
        """)
        header_layout.addWidget(title)
        header_layout.addStretch()

        layout.addLayout(header_layout)

        # 상태 라벨
        self.status_label = QLabel('Initializing ROS2...')
        self.status_label.setStyleSheet("color: #858585; font-size: 12px;")
        layout.addWidget(self.status_label)

        # 스크롤 영역
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QScrollBar:vertical {
                background-color: #1e1e1e;
                width: 12px;
            }
            QScrollBar::handle:vertical {
                background-color: #424242;
                border-radius: 6px;
                min-height: 20px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)

        # 프리뷰 그리드 컨테이너
        self.grid_container = QWidget()
        self.grid_container.setStyleSheet("background-color: transparent;")
        self.grid_layout = QGridLayout(self.grid_container)
        self.grid_layout.setContentsMargins(0, 0, 0, 0)
        self.grid_layout.setSpacing(16)

        scroll_area.setWidget(self.grid_container)
        layout.addWidget(scroll_area, 1)

    def _init_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init()

            self.ros_node = CameraSubscriberNode(self.image_signal)
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.ros_node)

            self._running = True
            self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self.ros_thread.start()

            self.status_label.setText('ROS2 initialized')
            self._discover_topics()

        except Exception as e:
            self.status_label.setText(f'ROS2 init failed: {e}')

    def _ros_spin(self):
        while self._running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def _discover_topics(self):
        if not self.ros_node:
            return

        topics = self.ros_node.get_available_image_topics()
        self.status_label.setText(f'Found {len(topics)} camera(s)')

        for topic in topics:
            self._add_preview(topic)

        self._update_grid_layout()

    def _add_preview(self, topic_name: str):
        widget = CameraPreviewWidget(topic_name)
        self.preview_widgets[topic_name] = widget

        if self.ros_node:
            self.ros_node.subscribe_to_topic(topic_name)

    def _update_grid_layout(self):
        """프리뷰 위젯들을 그리드에 배치 (2열)"""
        for idx, (topic, widget) in enumerate(self.preview_widgets.items()):
            row = idx // 2
            col = idx % 2
            self.grid_layout.addWidget(widget, row, col)

    def _on_image_received(self, topic_name: str, cv_image: np.ndarray):
        if topic_name in self.preview_widgets:
            self.preview_widgets[topic_name].update_image(cv_image)

    def cleanup(self):
        self._running = False

        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)

        if self.executor:
            self.executor.shutdown()

        if self.ros_node:
            self.ros_node.destroy_node()

        try:
            rclpy.shutdown()
        except:
            pass
