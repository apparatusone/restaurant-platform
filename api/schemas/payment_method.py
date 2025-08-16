from datetime import datetime
from pydantic import BaseModel
from enum import Enum
from typing import Optional
from decimal import Decimal

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
    check_id: int
    amount: Decimal
    payment_type: PaymentType = None
    status: PaymentStatus = PaymentStatus.PENDING
    card_number: Optional[str] = None
    order_id: Optional[int] = None  # optional reference for tracking


class PaymentCreate(PaymentBase):
    pass


class PaymentUpdate(BaseModel):
    check_id: Optional[int] = None
    amount: Optional[Decimal] = None
    payment_type: Optional[PaymentType] = None
    status: Optional[PaymentStatus] = None
    card_number: Optional[str] = None
    order_id: Optional[int] = None


class Payment(PaymentBase):
    id: int
    payment_date: datetime

    class ConfigDict:
        from_attributes = True


class SplitPaymentItem(BaseModel):
    amount: Decimal
    payment_type: PaymentType
    card_number: Optional[str] = None


class SplitPaymentRequest(BaseModel):
    payments: list[SplitPaymentItem]


class CheckPaymentCreate(BaseModel):
    """Schema for creating payments through the check endpoint"""
    amount: Decimal
    payment_type: PaymentType = PaymentType.CASH
    card_number: Optional[str] = None