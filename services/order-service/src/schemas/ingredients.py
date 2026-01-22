from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class IngredientBase(BaseModel):
    name: str
    quantity_on_hand: int
    restaurant_id: int
    unit: str
    reorder_point: int


class IngredientCreate(IngredientBase):
    pass


class IngredientUpdate(BaseModel):
    name: Optional[str] = None
    quantity_on_hand: Optional[int] = None
    restaurant_id: Optional[int] = None
    unit: Optional[str] = None
    reorder_point: Optional[int] = None


class Ingredient(IngredientBase):
    id: int

    class ConfigDict:
        from_attributes = True
