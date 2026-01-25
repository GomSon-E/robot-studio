from fastapi import APIRouter, HTTPException, Depends
from minio import Minio

from app.schemas.object import PresignedUploadUrlRequest, PresignedUrlResponse
from app.services.object_service import ObjectService
from app.infra.minio import get_minio_client

router = APIRouter(prefix="/objects", tags=["objects"])


def get_object_service(
    minio_client: Minio = Depends(get_minio_client)
) -> ObjectService:
    return ObjectService(minio_client)


@router.post("/presigned-upload-url", response_model=PresignedUrlResponse)
async def get_upload_url(
    request: PresignedUploadUrlRequest,
    service: ObjectService = Depends(get_object_service)
):
    try:
        url, expires_in = service.create_presigned_upload_url(
            object_key=request.object_name,
            expire_minutes=10
        )
        return PresignedUrlResponse(url=url, expires_in=expires_in)
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to generate presigned URL: {str(e)}"
        )
