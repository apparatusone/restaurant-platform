from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class RecipeBase(BaseModel):
    menu_item_id: int
    ingredient_id: int
    amount: int


class RecipeCreate(RecipeBase):
    pass

class RecipeUpdate(BaseModel):
    ingredient_id: Optional[int] = None
    amount: Optional[int] = None
    menu_item_id: Optional[int] = None

class Recipe(RecipeBase):
    id: int

    class ConfigDict:
        from_attributes = True
