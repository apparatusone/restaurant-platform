from sqlalchemy import Column, Integer, String, Boolean, Text
from sqlalchemy.orm import relationship
from shared.dependencies.database import Base


class Table(Base):
    """
    Represents physical tables in the restaurant.
    Simple and persistent - no timestamps needed.
    """
    __tablename__ = "tables"

    id = Column(Integer, primary_key=True, autoincrement=True)
    code = Column(String(8), unique=True, nullable=False, index=True)
    capacity = Column(Integer, nullable=False)
    section = Column(String(32), nullable=True)
    is_outdoor = Column(Boolean, default=False, nullable=False)
    notes = Column(Text, nullable=True)

    # relationships
    sessions = relationship("TableSeating", back_populates="table")
