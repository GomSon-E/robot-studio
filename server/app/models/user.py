import uuid
from datetime import datetime
from typing import Optional

from sqlalchemy import String, ForeignKey, Text
from sqlalchemy.orm import Mapped, mapped_column, relationship

from app.infra.database import Base


class User(Base):
    __tablename__ = "users"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    email: Mapped[str] = mapped_column(String(255), unique=True, nullable=False)
    password_hash: Mapped[str] = mapped_column(String(255), nullable=False)
    username: Mapped[Optional[str]] = mapped_column(String(100))
    plan: Mapped[str] = mapped_column(String(20), default="free")
    created_at: Mapped[datetime] = mapped_column(default=datetime.now)
    updated_at: Mapped[datetime] = mapped_column(default=datetime.now, onupdate=datetime.now)

    tokens: Mapped[list["UserToken"]] = relationship(back_populates="user", cascade="all, delete-orphan")
    api_credentials: Mapped[list["ApiCredential"]] = relationship(back_populates="user", cascade="all, delete-orphan")


class UserToken(Base):
    __tablename__ = "user_tokens"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    user_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    token_type: Mapped[Optional[str]] = mapped_column(String(50))
    token_hash: Mapped[str] = mapped_column(String(255), nullable=False)
    name: Mapped[Optional[str]] = mapped_column(String(100))
    expires_at: Mapped[Optional[datetime]] = mapped_column()
    created_at: Mapped[datetime] = mapped_column(default=datetime.now)

    user: Mapped["User"] = relationship(back_populates="tokens")


class ApiCredential(Base):
    __tablename__ = "api_credentials"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    user_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    service: Mapped[Optional[str]] = mapped_column(String(50))
    encrypted_token: Mapped[Optional[str]] = mapped_column(Text)
    created_at: Mapped[datetime] = mapped_column(default=datetime.now)

    user: Mapped["User"] = relationship(back_populates="api_credentials")
