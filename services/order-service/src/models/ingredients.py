from sqlalchemy import Column, Integer, String, Numeric
from sqlalchemy.orm import relationship
from shared.dependencies.database import Base


class Ingredient(Base):
    __tablename__ = "ingredients"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    restaurant_id = Column(Integer, nullable=False, index=True)
    name = Column(String(100), nullable=False)
    unit = Column(String(20), nullable=False)  # oz, lb, count, etc
    quantity_on_hand = Column(Numeric(10, 2), nullable=False, server_default='0.0')
    reorder_point = Column(Numeric(10, 2), nullable=False, server_default='0.0')
    apriltag_id = Column(Integer, nullable=True)  # For robot picking

    # relationships
    recipes = relationship("Recipe", back_populates="ingredient")
