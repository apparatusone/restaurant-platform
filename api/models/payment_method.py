from sqlalchemy import Column, ForeignKey, Integer, String, DECIMAL, DATETIME, Float, Enum
from sqlalchemy.orm import relationship
from datetime import datetime
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

        id = Column(Integer, primary_key=True, index=True, autoincrement=True)
        order_id = Column(Integer, ForeignKey("orders.id"), nullable=False)
        payment_date = Column(DATETIME, nullable=False, default=str(datetime.now()))
        amount = Column(Float, nullable=False)
        status = Column(Enum(PaymentStatus), nullable=False, default=PaymentStatus.PENDING)
        payment_type = Column(Enum(PaymentType), nullable=False)

        order = relationship("Order", back_populates="payment")