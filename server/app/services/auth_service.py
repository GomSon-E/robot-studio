import hashlib
import datetime
from datetime import datetime, timedelta

from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.core.security import hash_password, create_access_token, verify_password, create_refresh_token
from app.models.user import User, UserToken

class AuthService:
    def __init__(self, db: AsyncSession):
        self.db = db

    def _hash_token(self, token: str) -> str:
        """refresh token을 SHA-256으로 해시"""
        return hashlib.sha256(token.encode()).hexdigest()

    async def _issue_tokens(self, user: User) -> tuple[str, str]:
        access_token = create_access_token(str(user.id))
        refresh_token = create_refresh_token()

        # DB에 refresh token 해시 저장
        user_token = UserToken(
            user_id=user.id,
            token_type="refresh",
            token_hash=self._hash_token(refresh_token),
            expires_at=datetime.now() + timedelta(days=7),
        )
        self.db.add(user_token)
        await self.db.commit()

        return access_token, refresh_token

    async def signup(self, username: str, email: str, password: str) -> User:
        result = await self.db.execute(select(User).where(User.email == email))
        if result.scalar_one_or_none():
            raise ValueError("이미 등록된 이메일입니다")

        user = User(
            username=username,
            email=email,
            password_hash=hash_password(password),
        )
        self.db.add(user)
        await self.db.commit()

        return user

    async def login(self, email: str, password: str) -> tuple[str, str]:
        result = await self.db.execute(select(User).where(User.email == email))
        user = result.scalar_one_or_none()
        if not user or not verify_password(password, user.password_hash):
            raise ValueError("이메일 또는 비밀번호가 올바르지 않습니다")

        return await self._issue_tokens(user)

    async def refresh(self, refresh_token: str) -> tuple[str, str]:
        token_hash = self._hash_token(refresh_token)

        # 1. DB에서 refresh token 조회
        result = await self.db.execute(
            select(UserToken).where(
                UserToken.token_hash == token_hash,
                UserToken.token_type == "refresh",
            )
        )
        stored_token = result.scalar_one_or_none()

        if not stored_token:
            raise ValueError("유효하지 않은 refresh token입니다")

        # 2. 만료 확인
        if stored_token.expires_at and stored_token.expires_at < datetime.now():
            await self.db.delete(stored_token)
            await self.db.commit()
            raise ValueError("만료된 refresh token입니다")

        # 3. 기존 refresh token 삭제
        await self.db.delete(stored_token)

        # 4. 유저 조회 후 새 토큰 쌍 발급
        user_result = await self.db.execute(
            select(User).where(User.id == stored_token.user_id)
        )
        user = user_result.scalar_one_or_none()
        if not user:
            raise ValueError("사용자를 찾을 수 없습니다")

        return await self._issue_tokens(user)
