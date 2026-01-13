from sqlalchemy import Column, Integer, String
from sqlalchemy.orm import relationship
from shared.dependencies.database import Base


class Resource(Base):
    __tablename__ = "resources"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    item = Column(String(100), unique=True, nullable=False)
    amount = Column(Integer, index=True, nullable=False, server_default='0.0')

    menu_item_ingredients = relationship("MenuItemIngredient", back_populates="resource")
