from datetime import datetime
from typing import Optional
from pydantic import BaseModel
from enum import Enum
from .order_details import OrderDetail
from .payment_method import Payment


class OrderType(str, Enum):
    DINE_IN = "dine_in"
    TAKEOUT = "takeout"
    DELIVERY = "delivery"

class StatusType(str, Enum):
    PENDING = "pending"
    CONFIRMED = "confirmed"
    IN_PROGRESS = "in_progress"
    AWAITING_PICKUP = "awaiting_pickup"
    OUT_FOR_DELIVERY = "out_for_delivery"
    CANCELLED = "cancelled"
    COMPLETED = "completed"


class OrderBase(BaseModel):
    customer_id: Optional[int] = None
    description: Optional[str] = None
    status: StatusType = StatusType.PENDING
    order_type: OrderType = OrderType.DINE_IN
    promo_id: Optional[int] = None
    paid: bool = False
    tracking_number: Optional[str] = None


class OrderCreate(OrderBase):
    pass


class OrderUpdate(BaseModel):
    customer_id: Optional[int] = None
    description: Optional[str] = None
    status: Optional[StatusType] = None
    order_type: Optional[OrderType] = None
    promo_id: Optional[int] = None
    paid: Optional[bool] = None
    tracking_number: Optional[str] = None
    order_date: Optional[datetime] = None


class Order(OrderBase):
    id: int
    order_date: Optional[datetime] = None
    order_details: Optional[list[OrderDetail]] = None
    payment: Optional[Payment] = None

    class ConfigDict:
        from_attributes = True
