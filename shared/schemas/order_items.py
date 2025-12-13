from typing import Optional
from pydantic import BaseModel, Field, ConfigDict
from decimal import Decimal
from datetime import datetime
from enum import Enum


class OrderItemStatus(str, Enum):
    UNSENT = "unsent"
    SENT = "sent"
    READY = "ready"


class OrderItemBase(BaseModel):
    order_id: int = Field(..., description="FK -> orders.id")
    menu_item_id: int = Field(..., description="FK -> menu_items.id")
    quantity: int = Field(..., ge=1, description="Quantity ordered")
    unit_price: Decimal = Field(..., ge=0, description="Price per item at time of order")
    special_instructions: Optional[str] = Field(None, description="Special preparation notes")
    status: OrderItemStatus = Field(OrderItemStatus.UNSENT, description="Current status of the order item")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class OrderItemCreate(BaseModel):
    """Schema for creating order items - unit_price is automatically set from menu item"""
    order_id: int = Field(..., description="FK -> orders.id")
    menu_item_id: int = Field(..., description="FK -> menu_items.id")
    quantity: int = Field(..., ge=1, description="Quantity ordered")
    special_instructions: Optional[str] = Field(None, description="Special preparation notes")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class OrderItemUpdate(BaseModel):
    quantity: Optional[int] = Field(None, ge=1)
    special_instructions: Optional[str] = None
    status: Optional[OrderItemStatus] = None
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class CheckItemCreate(BaseModel):
    """Schema for adding menu items directly to a check"""
    menu_item_id: int = Field(..., description="FK -> menu_items.id")
    quantity: int = Field(..., ge=1, description="Quantity ordered")
    special_instructions: Optional[str] = Field(None, description="Special preparation notes")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class OrderItem(OrderItemBase):
    id: int
    line_total: Decimal = Field(..., description="quantity * unit_price")
    created_at: datetime
    updated_at: datetime
    
    model_config = ConfigDict(from_attributes=True)