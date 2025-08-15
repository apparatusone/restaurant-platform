from typing import Optional
from pydantic import BaseModel, Field, ConfigDict
from decimal import Decimal
from datetime import datetime


class OrderDetailBase(BaseModel):
    order_id: int = Field(..., description="FK -> orders.id")
    menu_item_id: int = Field(..., description="FK -> menu_items.id")
    quantity: int = Field(..., ge=1, description="Quantity ordered")
    unit_price: Decimal = Field(..., ge=0, description="Price per item at time of order")
    special_instructions: Optional[str] = Field(None, description="Special preparation notes")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class OrderDetailCreate(OrderDetailBase):
    pass


class OrderDetailUpdate(BaseModel):
    quantity: Optional[int] = Field(None, ge=1)
    special_instructions: Optional[str] = None
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class OrderDetail(OrderDetailBase):
    id: int
    line_total: Decimal = Field(..., description="quantity * unit_price")
    created_at: datetime
    updated_at: datetime
    
    model_config = ConfigDict(from_attributes=True)