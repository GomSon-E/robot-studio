from PySide6.QtCore import QThread, Signal

from .api_client import ApiClient


class PresignedUrlWorker(QThread):
    """Presigned URL 요청을 백그라운드에서 처리하는 워커"""

    urls_received = Signal(list)  # URL 리스트 반환
    error = Signal(str)           # 에러 메시지

    def __init__(self, api_client: ApiClient, settings: dict, parent=None):
        super().__init__(parent)
        self.api_client = api_client
        self.settings = settings

    def run(self):
        try:
            urls = self.api_client.get_presigned_urls(
                topic=self.settings['topic'],
                episodes=self.settings['episodes']
            )
            self.urls_received.emit(urls)
        except Exception as e:
            self.error.emit(str(e))
