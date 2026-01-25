from datetime import timedelta
from minio import Minio

from app.core.config import settings

class ObjectService:
    def __init__(self, minio_client: Minio):
        self.client = minio_client

    def create_presigned_upload_url(
        self,
        object_key: str,
        expire_minutes: int | None = None
    ) -> tuple[str, int]:
        expires = timedelta(minutes=expire_minutes)

        url = self.client.presigned_put_object(
            bucket_name=settings.MINIO_BUCKET_NAME,
            object_name=object_key,
            expires=expires,
        )

        return url, int(expires.total_seconds())
