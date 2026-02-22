from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.models.user import User, UserToken


class UserRepository:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def find_by_email(self, email: str) -> User | None:
        result = await self.db.execute(select(User).where(User.email == email))
        return result.scalar_one_or_none()

    async def find_by_id(self, user_id) -> User | None:
        result = await self.db.execute(select(User).where(User.id == user_id))
        return result.scalar_one_or_none()

    async def save(self, user: User) -> User:
        self.db.add(user)
        return user


class UserTokenRepository:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def find_by_hash(self, token_hash: str, token_type: str) -> UserToken | None:
        result = await self.db.execute(
            select(UserToken).where(
                UserToken.token_hash == token_hash,
                UserToken.token_type == token_type,
            )
        )
        return result.scalar_one_or_none()

    async def save(self, token: UserToken) -> UserToken:
        self.db.add(token)
        return token

    async def delete(self, token: UserToken) -> None:
        await self.db.delete(token)
