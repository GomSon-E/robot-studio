import asyncio
from rclpy.logging import get_logger

from ..utils.api_client import ApiClient

logger = get_logger('UploadService')


class UploadService:
    """S3 업로드 서비스 (presigned URL 요청 + 업로드 + 재시도)"""

    def __init__(self, api_client: ApiClient, max_retries: int = 3):
        self.api_client = api_client
        self.max_retries = max_retries

    async def upload_with_retry(self, video_path: str, object_name: str) -> bool:
        for attempt in range(self.max_retries):
            try:
                presigned_url = await self.api_client.get_presigned_url(object_name)
                await self._upload_video(video_path, presigned_url)
                return True

            except Exception as e:
                logger.warning(
                    f"Upload attempt {attempt + 1}/{self.max_retries} failed "
                    f"for '{object_name}': {e}"
                )
                if attempt < self.max_retries - 1:
                    delay = 2 ** attempt
                    logger.info(f"Retrying in {delay}s...")
                    await asyncio.sleep(delay)

        logger.error(f"Upload failed after {self.max_retries} attempts: '{object_name}'")
        return False

    async def _upload_video(self, video_path: str, presigned_url: str):
        """Presigned URL로 비디오 파일 업로드"""
        with open(video_path, 'rb') as f:
            video_data = f.read()

        async with self.api_client.session.put(
            presigned_url,
            data=video_data,
            headers={'Content-Type': 'video/mp4'},
        ) as response:
            if response.status == 200:
                logger.info(f"Upload successful: {presigned_url[:50]}...")
            else:
                response.raise_for_status()
