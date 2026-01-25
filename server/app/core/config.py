from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    MINIO_ENDPOINT: str = "localhost:9000"
    MINIO_ACCESS_KEY: str
    MINIO_SECRET_KEY: str
    MINIO_SECURE: bool
    MINIO_BUCKET_NAME: str

    PRESIGNED_URL_EXPIRE_MINUTES: int = 10
    
    class Config:
        env_file = ".env"

settings = Settings()