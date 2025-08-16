from sqlalchemy import Column, ForeignKey, Integer, String, DateTime, Enum, Numeric
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..dependencies.database import Base
import enum


class PaymentType(enum.Enum):
        CASH = "cash"
        CREDIT_CARD = "credit_card"
        DEBIT_CARD = "debit_card"


class PaymentStatus(enum.Enum):
        PENDING = "pending"
        COMPLETED = "completed"
        FAILED = "failed"
        REFUNDED = "refunded"


class Payment(Base):
        __tablename__ = "payment_method"

        id = Column(Integer, primary_key=True, autoincrement=True)
        check_id = Column(Integer, ForeignKey("checks.id", ondelete="CASCADE"), nullable=False)
        order_id = Column(Integer, ForeignKey("orders.id", ondelete="CASCADE"), nullable=True)
        amount = Column(Numeric(10, 2), nullable=False)
        payment_date = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
        status = Column(Enum(PaymentStatus, values_callable=lambda obj: [e.value for e in obj]), default=PaymentStatus.PENDING, nullable=False)
        payment_type = Column(Enum(PaymentType, values_callable=lambda obj: [e.value for e in obj]), nullable=False)
        card_number = Column(String(20), nullable=True)
        
        # relationships
        check = relationship("Check", back_populates="payments")
        order = relationship("Order", back_populates="payments")