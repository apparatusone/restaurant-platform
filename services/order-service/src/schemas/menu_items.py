from typing import Optional
from pydantic import BaseModel
from enum import Enum

class FoodCategory(str, Enum):
    VEGETARIAN = "vegetarian"
    VEGAN = "vegan"
    GLUTEN_FREE = "gluten_free"
    REGULAR = "regular"


class MenuItemsBase(BaseModel):
    name: str
    description: Optional[str] = None
    price: float
    calories: int
    food_category: FoodCategory

class MenuItemsCreate(MenuItemsBase):
    pass


class ResourceRequirement(BaseModel):
    resource_name: str
    quantity: int


class MenuItemsCreateWithResources(MenuItemsBase):
    resources: list[ResourceRequirement]

class MenuItemsUpdate(BaseModel):
    name: str
    description: Optional[str] = None
    price: float
    calories: int
    food_category: FoodCategory

class MenuItems(MenuItemsBase):
    id: int

    class ConfigDict:
        from_attributes = True