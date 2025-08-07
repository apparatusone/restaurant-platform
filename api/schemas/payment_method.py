from datetime import datetime
from pydantic import BaseModel
from enum import Enum
from typing import Optional

class PaymentType(str, Enum):
    CASH = "cash"
    CREDIT_CARD = "credit_card"
    DEBIT_CARD = "debit_card"

class PaymentStatus(str, Enum):
    PENDING = "pending"
    COMPLETED = "completed"
    FAILED = "failed"
    REFUNDED = "refunded"


class PaymentBase(BaseModel):
    order_id: int
    payment_type: PaymentType = None
    status: PaymentStatus = PaymentStatus.PENDING
    card_number: Optional[str] = None


class PaymentCreate(PaymentBase):
    pass


class PaymentUpdate(BaseModel):
    order_id: int
    payment_type: PaymentType = None
    status: PaymentStatus = None
    card_number: Optional[str] = None


class Payment(PaymentBase):
    id: int
    payment_date: datetime

    class ConfigDict:
        from_attributes = True