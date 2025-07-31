from typing import Optional
from pydantic import BaseModel
from .menu_items import MenuItems


class OrderDetailBase(BaseModel):
    amount: int


class OrderDetailCreate(OrderDetailBase):
    order_id: int
    menu_item_id: int

class OrderDetailUpdate(BaseModel):
    order_id: Optional[int] = None
    menu_item_id: Optional[int] = None
    amount: Optional[int] = None


class OrderDetail(OrderDetailBase):
    id: int
    order_id: int
    menu_item_id: int
    menu_item: MenuItems = None

    class ConfigDict:
        from_attributes = True