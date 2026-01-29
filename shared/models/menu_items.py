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
    food_category = Column(Enum(FoodCategory, values_callable=lambda obj: [e.value for e in obj]), nullable=False, index=True, default=FoodCategory.REGULAR) # vegetarian, vegan, etc

    # check_items is guaranteed to exist (in shared)
    check_items = relationship(
        "CheckItem",
        back_populates="menu_item",
        cascade="all, delete-orphan"
    )