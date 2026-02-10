from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from app.infra.database import get_db
from app.services.auth_service import AuthService
from app.schemas.auth import SignupRequest, SignupResponse, TokenResponse, RefreshRequest, LoginRequest

router = APIRouter(prefix="/auth", tags=["auth"])

def get_auth_service(db: AsyncSession = Depends(get_db)) -> AuthService:
    return AuthService(db)

@router.post("/signup", response_model=SignupResponse, status_code=status.HTTP_201_CREATED)
async def signup(req: SignupRequest, service: AuthService = Depends(get_auth_service)):
    try:
        user = await service.signup(req.username, req.email, req.password)
        return SignupResponse(username=user.username, email=user.email)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))

@router.post("/login", response_model=TokenResponse)
async def login(req: LoginRequest, service: AuthService = Depends(get_auth_service)):
    try:
        token = await service.login(req.email, req.password)
        return TokenResponse(access_token=token)
    except ValueError as e:
        raise HTTPException(status_code=401, detail=str(e))

@router.post("/refresh", response_model=TokenResponse)
async def refresh(req: RefreshRequest, service: AuthService = Depends(get_auth_service)):
    try:
        access_token, refresh_token = await service.refresh(req.refresh_token)
        return TokenResponse(access_token=access_token, refresh_token=refresh_token)
    except ValueError as e:
        raise HTTPException(status_code=401, detail=str(e))
