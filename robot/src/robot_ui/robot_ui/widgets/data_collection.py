import asyncio
import tempfile
import cv2
import aiohttp
import numpy as np
from pathlib import Path
from typing import Optional
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QProgressBar
from PySide6.QtCore import Qt, Signal
from rclpy.logging import get_logger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

logger = get_logger('DataCollection')


class DataCollectionPanel(QWidget):
    """데이터 수집 패널"""

    recording_finished = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.settings: dict = {}
        self.presigned_urls: list = []
        self.is_recording = False
        self.collected_frames: list = []
        self.ros_node = None
        self.bridge = CvBridge()
        self._frame_subscription = None
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # 헤더
        title = QLabel('Data Collection')
        title.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-size: 18px;
                font-weight: 600;
            }
        """)
        layout.addWidget(title)

        # 상태 표시
        self.status_label = QLabel('Ready to record')
        self.status_label.setStyleSheet("color: #858585; font-size: 14px;")
        layout.addWidget(self.status_label)

        # 진행률 표시
        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #3c3c3c;
                border-radius: 4px;
                background-color: #2d2d2d;
                text-align: center;
                color: #ffffff;
            }
            QProgressBar::chunk {
                background-color: #0e639c;
                border-radius: 3px;
            }
        """)
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)

        layout.addStretch()

        # Record 버튼
        self.record_btn = QPushButton('Record')
        self.record_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.record_btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: #ffffff;
                border: none;
                border-radius: 4px;
                padding: 12px 32px;
                font-size: 16px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #e53935;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
            QPushButton:disabled {
                background-color: #5c5c5c;
            }
        """)
        self.record_btn.clicked.connect(self._on_record_clicked)
        layout.addWidget(self.record_btn, alignment=Qt.AlignmentFlag.AlignCenter)

        layout.addStretch()

    def set_ros_node(self, ros_node):
        """ROS2 노드 설정"""
        self.ros_node = ros_node

    def set_recording_config(self, settings: dict, presigned_urls: list):
        """녹화 설정 저장"""
        self.settings = settings
        self.presigned_urls = presigned_urls
        self.status_label.setText(
            f"Ready: {settings['episodes']} episodes, "
            f"{settings['data_length']}s each"
        )

    def _on_record_clicked(self):
        """Record 버튼 클릭 시"""
        if not self.is_recording:
            self.recodign_task = asyncio.create_task(self._start_recording())

    def _on_frame_received(self, msg: Image):
        """ROS2 토픽에서 프레임 수신"""
        if self.is_recording:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.collected_frames.append(cv_image.copy())
            except Exception as e:
                logger.error(f"Failed to convert frame: {e}")

    async def _start_recording(self):
        """녹화 시작"""
        if not self.presigned_urls:
            logger.error("No presigned URLs available")
            return

        if not self.ros_node:
            logger.error("ROS2 node not available")
            return

        self.is_recording = True
        self.record_btn.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setMaximum(len(self.presigned_urls))
        self.progress_bar.setValue(0)

        topic = self.settings.get('topic', '')
        data_length = self.settings.get('data_length', 10.0)
        term_length = self.settings.get('term_length', 1.0)

        # 토픽 구독
        self._frame_subscription = self.ros_node.create_subscription(
            Image, topic, self._on_frame_received, 10
        )

        for i, url_info in enumerate(self.presigned_urls):
            self.status_label.setText(f"Recording episode {i + 1}/{len(self.presigned_urls)}...")
            self.progress_bar.setValue(i)

            try:
                # 프레임 수집
                self.collected_frames = []
                await asyncio.sleep(data_length)

                # 영상 파일 생성
                if self.collected_frames:
                    video_path = self._save_video()

                    if video_path:
                        # MinIO에 업로드
                        self.status_label.setText(f"Uploading episode {i + 1}...")
                        await self._upload_video(video_path, url_info['url'])

                        # 임시 파일 삭제
                        Path(video_path).unlink(missing_ok=True)
                else:
                    logger.error(f"No frames collected for episode {i + 1}")

                # 다음 에피소드 전 대기 (마지막 제외)
                if i < len(self.presigned_urls) - 1 and term_length > 0:
                    self.status_label.setText(f"Waiting {term_length}s...")
                    await asyncio.sleep(term_length)

            except Exception as e:
                logger.error(f"Error recording episode {i + 1}: {e}")

        # 구독 해제
        if self._frame_subscription:
            self.ros_node.destroy_subscription(self._frame_subscription)
            self._frame_subscription = None

        self.progress_bar.setValue(len(self.presigned_urls))
        self.status_label.setText("Recording complete!")
        self.is_recording = False
        self.record_btn.setEnabled(True)
        self.recording_finished.emit()

    def _save_video(self) -> Optional[str]:
        """수집된 프레임을 비디오 파일로 저장"""
        if not self.collected_frames:
            return None

        fps = 30
        height, width = self.collected_frames[0].shape[:2]

        # 임시 파일 생성
        temp_file = tempfile.NamedTemporaryFile(suffix='.mp4', delete=False)
        video_path = temp_file.name
        temp_file.close()

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

        for frame in self.collected_frames:
            out.write(frame)

        out.release()

        logger.info(f"Saved video: {video_path} ({len(self.collected_frames)} frames)")
        return video_path

    async def _upload_video(self, video_path: str, presigned_url: str):
        """Presigned URL로 비디오 업로드"""
        with open(video_path, 'rb') as f:
            video_data = f.read()

        async with aiohttp.ClientSession() as session:
            async with session.put(
                presigned_url,
                data=video_data,
                headers={'Content-Type': 'video/mp4'}
            ) as response:
                if response.status == 200:
                    logger.info(f"Upload successful: {presigned_url[:50]}...")
                else:
                    logger.error(f"Upload failed: {response.status}")
