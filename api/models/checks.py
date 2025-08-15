from sqlalchemy import Column, Integer, String, DateTime, ForeignKey, Numeric, Enum, Boolean
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..dependencies.database import Base
import enum

class CheckStatus(enum.Enum):
    OPEN = "open"
    SUBMITTED = "submitted"
    PAYMENT_PENDING = "payment_pending"
    CLOSED = "closed"

class Check(Base):
    """
    Each check represents a separate bill within a table session or virtual check for online orders
    """
    __tablename__ = "checks"

    id = Column(Integer, primary_key=True, autoincrement=True)
    
    # link to table session (nullable for virtual checks)
    session_id = Column(Integer, ForeignKey('table_sessions.id'), nullable=True)
    
    # virtual check flag for online orders
    is_virtual = Column(Boolean, default=False, nullable=False)
    
    status = Column(Enum(CheckStatus), default=CheckStatus.OPEN, nullable=False)
    
    subtotal = Column(Numeric(10, 2), default=0.00)
    tax_amount = Column(Numeric(10, 2), default=0.00)
    tip_amount = Column(Numeric(10, 2), default=0.00)
    total_amount = Column(Numeric(10, 2), default=0.00)
    
    # timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
    submitted_at = Column(DateTime(timezone=True), nullable=True)
    paid_at = Column(DateTime(timezone=True), nullable=True)
    

    # relationships
    session = relationship("TableSession", back_populates="checks")
    orders = relationship("Order", back_populates="check", cascade="all, delete-orphan")