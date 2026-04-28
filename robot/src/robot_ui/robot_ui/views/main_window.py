import asyncio
from PySide6.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QStackedWidget
from PySide6.QtCore import Qt
from rclpy.logging import get_logger
from ..widgets import Sidebar, CameraPreviewArea, DatasetSettingPanel, DataCollectionPanel, TeleopPanel, LoginWebView, CalibrationPanel
from ..widgets.theme import BG_GRADIENT, TEXT_BODY, FONT_FAMILY
from ..utils.api_client import ApiClient

logger = get_logger('MainWindow')


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Robot Studio')
        self.setMinimumSize(1200, 800)

        self.setStyleSheet(f"""
            QWidget {{
                color: {TEXT_BODY};
                font-family: {FONT_FAMILY};
            }}
        """)

        self.api_client = ApiClient()
        self._setup_ui()

    def _setup_ui(self):
        central_widget = QWidget()
        central_widget.setObjectName('AppRoot')
        central_widget.setStyleSheet(f"#AppRoot {{ background: {BG_GRADIENT}; }}")
        central_widget.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setCentralWidget(central_widget)

        root_layout = QVBoxLayout(central_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        self.stacked_widget = QStackedWidget()
        root_layout.addWidget(self.stacked_widget)

        # 페이지 0: 로그인
        self.login_webview = LoginWebView()
        self.login_webview.login_success.connect(self._on_login_success)
        self.stacked_widget.addWidget(self.login_webview)  # index 0

        # 페이지 1: 메인
        main_page = QWidget()
        main_layout = QHBoxLayout(main_page)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 1. 고정 사이드바
        self.sidebar = Sidebar()
        self.sidebar.menu_selected.connect(self._on_menu_selected)
        self.sidebar.exit_requested.connect(self.close)
        main_layout.addWidget(self.sidebar)

        # 2. 캘리브레이션 패널
        self.calibration_panel = CalibrationPanel()
        self.calibration_panel.setVisible(False)
        main_layout.addWidget(self.calibration_panel, 1)

        # 3. 텔레옵 패널
        self.teleop_panel = TeleopPanel()
        self.teleop_panel.setVisible(False)
        main_layout.addWidget(self.teleop_panel, 1)

        # 3. 메인 콘텐츠 영역
        self.camera_preview_area = CameraPreviewArea()
        self.camera_preview_area.setVisible(False)
        self.camera_preview_area.camera_selected.connect(self._on_camera_selected)
        main_layout.addWidget(self.camera_preview_area, 1)

        # 3. 데이터셋 설정 패널
        self.dataset_setting_panel = DatasetSettingPanel()
        self.dataset_setting_panel.setVisible(False)
        self.dataset_setting_panel.submitted.connect(self._on_dataset_submitted)
        main_layout.addWidget(self.dataset_setting_panel, 1)

        # 4. 데이터 수집 패널
        self.data_collection_panel = DataCollectionPanel(api_client=self.api_client)
        self.data_collection_panel.setVisible(False)
        main_layout.addWidget(self.data_collection_panel, 1)

        # 빈 메인 영역 (기본)
        self.empty_area = QWidget()
        self.empty_area.setStyleSheet("background-color: transparent;")
        self.empty_area.setVisible(False)
        main_layout.addWidget(self.empty_area, 1)

        # 카메라 토픽 목록 → DatasetSettingPanel 콤보박스 + DataCollectionPanel 그리드 연결
        self.camera_preview_area.topics_updated.connect(self._on_topics_updated)
        # 초기화 시 이미 발견된 토픽 즉시 반영
        self.dataset_setting_panel.set_available_topics(
            list(self.camera_preview_area.preview_widgets.keys())
        )

        self.stacked_widget.addWidget(main_page)  # index 1
        self.stacked_widget.setCurrentIndex(0)

        # 기본 패널: Calibration
        self.calibration_panel.setVisible(True)
        self.sidebar._items['calibration'].setChecked(True)

    def _on_login_success(self, code: str):
        asyncio.create_task(self._exchange_and_login(code))

    async def _exchange_and_login(self, code: str):
        try:
            tokens = await self.api_client.exchange_code(code)
            self.api_client.set_token(tokens["access_token"], tokens["refresh_token"])
            self.stacked_widget.setCurrentIndex(1)
        except ValueError:
            logger.warning("Code exchange failed: invalid or expired code")
            self.login_webview.reset()
        except Exception as e:
            logger.error(f"Code exchange failed: {e}")
            self.login_webview.reset()

    def _on_topics_updated(self, topics: list):
        self.dataset_setting_panel.set_available_topics(topics)
        if self.data_collection_panel.isVisible():
            self.data_collection_panel.update_camera_roles(
                self.dataset_setting_panel.get_settings()['camera_roles']
            )


    def _on_menu_selected(self, menu_id: str):
        # 모든 영역 숨기기
        self.calibration_panel.setVisible(False)
        self.teleop_panel.setVisible(False)
        self.camera_preview_area.setVisible(False)
        self.dataset_setting_panel.setVisible(False)
        self.data_collection_panel.setVisible(False)
        self.empty_area.setVisible(False)

        if menu_id == 'calibration':
            self.calibration_panel.setVisible(True)
        elif menu_id == 'teleop':
            self.teleop_panel.setVisible(True)
        elif menu_id == 'camera_preview':
            self.camera_preview_area.setVisible(True)
        elif menu_id == 'dataset_setting':
            self.dataset_setting_panel.setVisible(True)
        else:
            self.empty_area.setVisible(True)

    def _on_camera_selected(self, topic_name: str):
        """카메라 선택 시 Dataset Setting 화면으로 전환"""
        # 모든 영역 숨기고 Dataset Setting 표시
        self.camera_preview_area.setVisible(False)
        self.dataset_setting_panel.setVisible(True)
        self.empty_area.setVisible(False)

        # 사이드바 메뉴 상태 업데이트
        self.sidebar._items['camera_preview'].setChecked(False)
        self.sidebar._items['dataset_setting'].setChecked(True)

    def _on_dataset_submitted(self, settings: dict):
        """Dataset Setting 제출 시 데이터 수집 화면으로 이동"""
        logger.info(f"Dataset settings submitted: {settings}")

        # ROS2 노드 공유
        if self.camera_preview_area.ros_node:
            self.data_collection_panel.set_ros_node(self.camera_preview_area.ros_node)
        self.data_collection_panel.set_recording_config(settings)
        self.camera_preview_area.setVisible(False)
        self.dataset_setting_panel.setVisible(False)
        self.data_collection_panel.setVisible(True)
        self.empty_area.setVisible(False)

    def closeEvent(self, event):
        if hasattr(self, 'calibration_panel'):
            self.calibration_panel.cleanup()
        if hasattr(self, 'teleop_panel'):
            self.teleop_panel.cleanup()
        if hasattr(self, 'camera_preview_area'):
            self.camera_preview_area.cleanup()
        super().closeEvent(event)
