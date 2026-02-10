import asyncio
import aiohttp
from typing import List


class ApiClient:
    """백엔드 API 클라이언트"""

    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url

    async def get_presigned_urls(self, topic: str, episodes: int) -> List[dict]:
        """에피소드 개수만큼 presigned URL 동시 요청"""
        safe_topic = topic.strip('/').replace('/', '_')

        async with aiohttp.ClientSession() as session:
            tasks = []
            for i in range(episodes):
                object_name = f"{safe_topic}/episode_{i:04d}.mp4"
                tasks.append(self._fetch_presigned_url(session, object_name))

            urls = await asyncio.gather(*tasks)

        return list(urls)

    async def _fetch_presigned_url(
        self, session: aiohttp.ClientSession, object_name: str
    ) -> dict:
        """단일 presigned URL 요청"""
        async with session.post(
            f"{self.base_url}/api/v1/objects/presigned-upload-url",
            json={"object_name": object_name},
            timeout=aiohttp.ClientTimeout(total=10)
        ) as response:
            response.raise_for_status()
            return await response.json()
