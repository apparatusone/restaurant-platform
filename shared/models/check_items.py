from sqlalchemy import Column, ForeignKey, Integer, Numeric, DateTime, Text, Enum
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..dependencies.database import Base
import enum


class CheckItemStatus(enum.Enum):
    PENDING = "pending"
    PREPARING = "preparing"
    READY = "ready"
    SERVED = "served"


class CheckItem(Base):
    __tablename__ = "check_items"

    id = Column(Integer, primary_key=True, autoincrement=True)
    check_id = Column(Integer, ForeignKey("checks.id", ondelete="CASCADE"), nullable=False, index=True)
    menu_item_id = Column(Integer, ForeignKey("menu_items.id"), nullable=False)
    quantity = Column(Integer, nullable=False)
    unit_price = Column(Numeric(10, 2), nullable=False)
    total_price = Column(Numeric(10, 2), nullable=False)
    special_instructions = Column(Text, nullable=True)
    status = Column(Enum(CheckItemStatus, values_callable=lambda obj: [e.value for e in obj]), default=CheckItemStatus.PENDING, nullable=False)
    
    # timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)

    # relationships
    check = relationship("Check", back_populates="check_items")
    menu_item = relationship("MenuItem", back_populates="check_items")
