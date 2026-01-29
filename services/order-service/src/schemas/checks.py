from pydantic import BaseModel, Field, ConfigDict
from typing import Optional
from datetime import datetime
from decimal import Decimal
from enum import Enum


class CheckStatus(str, Enum):
    OPEN = "open"
    SENT = "sent"
    READY = "ready"
    PAID = "paid"
    CLOSED = "closed"


class CheckBase(BaseModel):
    seating_id: Optional[int] = Field(None, description="FK -> table_seatings.id (nullable for virtual checks)")
    is_virtual: bool = Field(False, description="True for online orders without table seating")
    subtotal: Optional[Decimal] = Field(Decimal('0.00'), ge=0, description="Subtotal amount")
    tax_amount: Optional[Decimal] = Field(Decimal('0.00'), ge=0, description="Tax amount")
    tip_amount: Optional[Decimal] = Field(Decimal('0.00'), ge=0, description="Tip amount")
    total_amount: Optional[Decimal] = Field(Decimal('0.00'), ge=0, description="Total amount")

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class CheckCreate(CheckBase):
    pass


class CheckUpdate(BaseModel):
    seating_id: Optional[int] = Field(None, description="FK -> table_seatings.id (nullable for virtual checks)")
    is_virtual: Optional[bool] = Field(None, description="True for online orders without table seating")
    status: Optional[CheckStatus] = None
    subtotal: Optional[Decimal] = Field(None, ge=0)
    tax_amount: Optional[Decimal] = Field(None, ge=0)
    tip_amount: Optional[Decimal] = Field(None, ge=0)
    total_amount: Optional[Decimal] = Field(None, ge=0)
    submitted_at: Optional[datetime] = None
    paid_at: Optional[datetime] = None

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class Check(CheckBase):
    id: int
    status: CheckStatus
    created_at: datetime
    updated_at: datetime
    submitted_at: Optional[datetime] = None
    paid_at: Optional[datetime] = None
    
    model_config = ConfigDict(from_attributes=True)


class CheckWithSession(Check):
    """Extended version with session details"""
    table_codes: list[str] = Field(default_factory=list, description="Table codes for this session")
    
    model_config = ConfigDict(from_attributes=True)