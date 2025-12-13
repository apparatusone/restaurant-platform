from sqlalchemy import Column, Integer, String, Numeric, Enum
from sqlalchemy.orm import relationship
from ..dependencies.database import Base
import enum

class FoodCategory(enum.Enum):
    VEGETARIAN = "vegetarian"
    VEGAN = "vegan"
    GLUTEN_FREE = "gluten_free"
    REGULAR = "regular"

class MenuItem(Base):
    __tablename__ = "menu_items"

    id            = Column(Integer, primary_key=True, autoincrement=True)
    name          = Column(String(255), unique=True, nullable=False, index=True)  # dish name
    description   = Column(String(255), nullable=True)
    price         = Column(Numeric(10, 2), nullable=False)  # price in dollars and cents
    calories      = Column(Integer, nullable=False)
    food_category = Column(Enum(FoodCategory), nullable=False, index=True, default=FoodCategory.REGULAR) # vegetarian, vegan, etc

    # order_items is guaranteed to exist (in shared)
    order_items = relationship(
        "OrderItem",
        back_populates="menu_item",
        cascade="all, delete-orphan"
    )
    
    # Note: menu_item_ingredients, reviews relationships are added dynamically
    # in order-service's model_loader to avoid import issues in kitchen-service