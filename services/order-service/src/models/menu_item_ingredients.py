from sqlalchemy import Column, ForeignKey, Integer
from sqlalchemy.orm import relationship
from shared.dependencies.database import Base


class MenuItemIngredient(Base):
    __tablename__ = "menu_item_ingredients"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    menu_item_id = Column(Integer, ForeignKey("menu_items.id"))
    resource_id = Column(Integer, ForeignKey("resources.id"))
    amount = Column(Integer, nullable=False, server_default='0.0')

    menu_item = relationship("MenuItem", back_populates="menu_item_ingredients")
    resource = relationship("Resource", back_populates="menu_item_ingredients")