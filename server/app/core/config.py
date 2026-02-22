from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    S3_BUCKET_NAME: str = "lerobot-soarm-dataset"
    PRESIGNED_URL_EXPIRE_MINUTES: int = 10

    class Config:
        env_file = ".env"

settings = Settings()
