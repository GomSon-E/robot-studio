from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    #MINIO CONFIGURATION
    MINIO_ENDPOINT: str = "localhost:9000"
    MINIO_ACCESS_KEY: str
    MINIO_SECRET_KEY: str
    MINIO_SECURE: bool
    MINIO_BUCKET_NAME: str
    PRESIGNED_URL_EXPIRE_MINUTES: int = 10

    #DATABASE CONFIGURATION
    DATABASE_URL: str = "postgresql+asyncpg://robot_studio:robot_studio@localhost:5432/robot_studio"
    JWT_SECRET_KEY: str = "change-this-to-random-secret"
    JWT_ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60
    
    class Config:
        env_file = ".env"

settings = Settings()