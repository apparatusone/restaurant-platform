from typing import Optional
from pydantic import BaseModel, Field, ConfigDict
from decimal import Decimal
from datetime import datetime
from enum import Enum


class CheckItemStatus(str, Enum):
    PENDING = "pending"
    PREPARING = "preparing"
    READY = "ready"
    SERVED = "served"


class CheckItemBase(BaseModel):
    check_id: int = Field(..., description="FK -> checks.id")
    menu_item_id: int = Field(..., description="FK -> menu_items.id")
    quantity: int = Field(..., ge=1, description="Quantity ordered")
    unit_price: Decimal = Field(..., ge=0, description="Price per item at time of order")
    special_instructions: Optional[str] = Field(None, description="Special preparation notes")
    status: CheckItemStatus = Field(CheckItemStatus.PENDING, description="Current status of the check item")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class CheckItemCreate(BaseModel):
    """Schema for creating check items - unit_price is automatically set from menu item"""
    check_id: int = Field(..., description="FK -> checks.id")
    menu_item_id: int = Field(..., description="FK -> menu_items.id")
    quantity: int = Field(..., ge=1, description="Quantity ordered")
    special_instructions: Optional[str] = Field(None, description="Special preparation notes")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class CheckItemUpdate(BaseModel):
    quantity: Optional[int] = Field(None, ge=1)
    special_instructions: Optional[str] = None
    status: Optional[CheckItemStatus] = None
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class CheckItemCreateDirect(BaseModel):
    """Schema for adding menu items directly to a check"""
    menu_item_id: int = Field(..., description="FK -> menu_items.id")
    quantity: int = Field(..., ge=1, description="Quantity ordered")
    special_instructions: Optional[str] = Field(None, description="Special preparation notes")
    
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class CheckItem(CheckItemBase):
    id: int
    total_price: Decimal = Field(..., description="quantity * unit_price")
    created_at: datetime
    updated_at: datetime
    
    model_config = ConfigDict(from_attributes=True)
