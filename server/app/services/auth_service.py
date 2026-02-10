from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from app.core.security import hash_password, create_access_token, verify_password
from app.models.user import User

class AuthService:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def signup(self, username: str, email: str, password: str) -> str:
        # 1. 이메일 중복 검사
        result = await self.db.execute(select(User).where(User.email == email))
        if result.scalar_one_or_none():
            raise ValueError("이미 등록된 이메일입니다")

        # 2. 유저 생성
        user = User(
            username=username,
            email=email,
            password_hash=hash_password(password),
        )
        self.db.add(user)
        await self.db.commit()

        # 3. JWT 발급
        return create_access_token(str(user.id))
    
    async def login(self, email: str, password: str) -> str:
        # 1. 이메일로 유저 조회
        result = await self.db.execute(select(User).where(User.email == email))
        user = result.scalar_one_or_none()
        if not user or not verify_password(password, user.password_hash):
            raise ValueError("이메일 또는 비밀번호가 올바르지 않습니다")

        # 2. JWT 발급
        return create_access_token(str(user.id))