from datetime import datetime
from pydantic import BaseModel
from typing import Optional
from decimal import Decimal
from ..models.payment import PaymentType, PaymentStatus


class PaymentBase(BaseModel):
    check_id: int
    amount: Decimal
    payment_type: PaymentType = None
    status: PaymentStatus = PaymentStatus.PENDING
    card_number: Optional[str] = None


class PaymentCreate(PaymentBase):
    pass


class PaymentUpdate(BaseModel):
    check_id: Optional[int] = None
    amount: Optional[Decimal] = None
    payment_type: Optional[PaymentType] = None
    status: Optional[PaymentStatus] = None
    card_number: Optional[str] = None


class Payment(PaymentBase):
    id: int
    payment_date: datetime

    class ConfigDict:
        from_attributes = True

class CheckPaymentCreate(BaseModel):
    """Schema for creating payments through the check endpoint"""
    amount: Decimal
    payment_type: PaymentType = PaymentType.CASH
    card_number: Optional[str] = None
