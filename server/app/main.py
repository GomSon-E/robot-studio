from contextlib import asynccontextmanager

from fastapi import FastAPI

app = FastAPI(title="Robot Studio API")

app.include_router(objects.router, prefix="/api/v1")
app.include_router(auth.router, prefix="/api/v1")

@app.get("/health")
async def health_check():
    return {"status": "healthy"}
