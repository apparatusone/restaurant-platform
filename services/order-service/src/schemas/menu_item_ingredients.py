from datetime import datetime
from typing import Optional
from pydantic import BaseModel
from .resources import Resource


class MenuItemIngredientBase(BaseModel):
    menu_item_id: int
    resource_id: int
    amount: int


class MenuItemIngredientCreate(MenuItemIngredientBase):
    pass

class MenuItemIngredientUpdate(BaseModel):
    resource_id: Optional[int] = None
    amount: Optional[int] = None
    menu_item_id: Optional[int] = None

class MenuItemIngredient(MenuItemIngredientBase):
    id: int
    resource: Resource = None

    class ConfigDict:
        from_attributes = True