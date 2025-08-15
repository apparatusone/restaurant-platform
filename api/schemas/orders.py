from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field, ConfigDict
from enum import Enum
from .order_details import OrderDetail


class OrderType(str, Enum):
    DINE_IN = "dine_in"
    TAKEOUT = "takeout"
    DELIVERY = "delivery"


class OrderStatus(str, Enum):
    # kitchen workflow
    PENDING = "pending"
    CONFIRMED = "confirmed"
    IN_PROGRESS = "in_progress"
    READY = "ready"
    
    # fulfillment workflow
    AWAITING_PICKUP = "awaiting_pickup"
    OUT_FOR_DELIVERY = "out_for_delivery"
    DELIVERED = "delivered"
    SERVED = "served"  # for dine-in
    COMPLETED = "completed"
    CANCELLED = "cancelled"


class OrderBase(BaseModel):
    check_id: int = Field(..., description="FK -> checks.id")
    customer_id: Optional[int] = Field(None, description="FK -> customers.id")
    promo_id: Optional[int] = Field(None, description="FK -> promotions.id")
    status: OrderStatus = OrderStatus.PENDING
    order_type: OrderType = OrderType.DINE_IN
    tracking_number: Optional[str] = Field(None, description="Tracking number for takeout/delivery orders")
    notes: Optional[str] = Field(None, description="Special instructions")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class OrderCreate(BaseModel):
    # check_id is optional for online orders - will be auto-created
    check_id: Optional[int] = Field(None, description="FK -> checks.id (optional for online orders)")
    customer_id: Optional[int] = Field(None, description="FK -> customers.id")
    promo_id: Optional[int] = Field(None, description="FK -> promotions.id")
    status: OrderStatus = OrderStatus.PENDING
    order_type: OrderType = OrderType.DINE_IN
    tracking_number: Optional[str] = Field(None, description="Tracking number for takeout/delivery orders")
    notes: Optional[str] = Field(None, description="Special instructions")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class OrderUpdate(BaseModel):
    customer_id: Optional[int] = None
    promo_id: Optional[int] = None
    status: Optional[OrderStatus] = None
    order_type: Optional[OrderType] = None
    tracking_number: Optional[str] = None
    notes: Optional[str] = None
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class Order(OrderBase):
    id: int
    order_date: datetime
    created_at: datetime
    updated_at: datetime
    order_details: Optional[list[OrderDetail]] = None
    
    model_config = ConfigDict(from_attributes=True)
