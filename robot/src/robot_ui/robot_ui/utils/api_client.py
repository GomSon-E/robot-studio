import requests
from typing import List


class ApiClient:
    """백엔드 API 클라이언트"""

    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url

    def get_presigned_urls(self, topic: str, episodes: int) -> List[dict]:
        """에피소드 개수만큼 presigned URL 요청"""
        urls = []
        # 토픽 이름에서 슬래시를 언더스코어로 변환
        safe_topic = topic.strip('/').replace('/', '_')

        for i in range(episodes):
            object_name = f"{safe_topic}/episode_{i:04d}.mp4"
            response = requests.post(
                f"{self.base_url}/api/v1/objects/presigned-upload-url",
                json={"object_name": object_name},
                timeout=10
            )
            response.raise_for_status()
            urls.append(response.json())

        return urls
