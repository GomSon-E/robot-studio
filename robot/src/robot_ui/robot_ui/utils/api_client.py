import aiohttp


class ApiClient:
    """백엔드 API 클라이언트"""

    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url

    async def get_presigned_url(self, object_name: str) -> str:
        """단일 presigned URL 요청"""
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{self.base_url}/api/v1/objects/presigned-upload-url",
                json={"object_name": object_name},
                timeout=aiohttp.ClientTimeout(total=10)
            ) as response:
                response.raise_for_status()
                data = await response.json()
                return data['url']
