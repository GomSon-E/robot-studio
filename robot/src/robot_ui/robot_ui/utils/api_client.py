import aiohttp


class ApiClient:
    """백엔드 API 클라이언트"""

    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url
        self.session = aiohttp.ClientSession()

    async def close(self):
        await self.session.close()

    async def get_presigned_url(self, object_name: str) -> str:
        """단일 presigned URL 요청"""
        async with self.session.post(
            f"{self.base_url}/api/v1/objects/presigned-upload-url",
            json={"object_name": object_name},
            timeout=aiohttp.ClientTimeout(total=10)
        ) as response:
            response.raise_for_status()
            data = await response.json()
            return data['url']

    async def upload_to_s3(self, presigned_url: str, video_path: str):
        """Presigned URL로 파일 업로드"""
        with open(video_path, 'rb') as f:
            video_data = f.read()

        async with self.session.put(
            presigned_url,
            data=video_data,
            headers={'Content-Type': 'video/mp4'},
        ) as response:
            response.raise_for_status()
