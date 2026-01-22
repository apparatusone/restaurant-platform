from sqlalchemy import Column, Integer, DateTime, Numeric, Enum, ForeignKey, Boolean
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from shared.dependencies.database import Base
import enum

class CheckStatus(enum.Enum):
    OPEN = "open"
    SENT = "sent"
    READY = "ready"
    PAID = "paid"
    CLOSED = "closed"

class Check(Base):
    """
    Guest check/bill
    """
    __tablename__ = "checks"

    id = Column(Integer, primary_key=True, autoincrement=True)
    
    # Session reference (nullable for virtual/online orders)
    session_id = Column(Integer, ForeignKey('table_sessions.id'), nullable=True)
    is_virtual = Column(Boolean, default=False, nullable=False)
    
    # Multi-tenant support
    restaurant_id = Column(Integer, nullable=True, index=True)
    
    # check status: open -> sent -> ready -> paid -> closed
    status = Column(Enum(CheckStatus, values_callable=lambda obj: [e.value for e in obj]), default=CheckStatus.OPEN, nullable=False)
    
    subtotal = Column(Numeric(10, 2), default=0.00)
    tax_amount = Column(Numeric(10, 2), default=0.00)
    tip_amount = Column(Numeric(10, 2), default=0.00)
    total_amount = Column(Numeric(10, 2), default=0.00)
    
    # timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)
    submitted_at = Column(DateTime(timezone=True), nullable=True)
    closed_at = Column(DateTime(timezone=True), nullable=True)
    paid_at = Column(DateTime(timezone=True), nullable=True)
    
    # relationships
    session = relationship("TableSession", back_populates="checks")
    check_items = relationship("CheckItem", back_populates="check", cascade="all, delete-orphan")
    payments = relationship("Payment", back_populates="check", cascade="all, delete-orphan")