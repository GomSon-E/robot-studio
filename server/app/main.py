from fastapi import FastAPI
from app.api.v1 import objects

app = FastAPI(title="Robot Studio API")

app.include_router(objects.router, prefix="/api/v1")

@app.get("/health")
async def health_check():
    return {"status": "healthy"}
