from sqlalchemy import Column, Integer
from sqlalchemy.orm import relationship
from shared.dependencies.database import Base


class Recipe(Base):
    __tablename__ = "recipes"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    menu_item_id = Column(Integer)
    ingredient_id = Column(Integer)
    amount = Column(Integer, nullable=False, server_default='0.0')
