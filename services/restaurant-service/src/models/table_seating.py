from sqlalchemy import Column, Integer, String, DateTime, ForeignKey, Text
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from shared.dependencies.database import Base

class TableSeating(Base):
    """
    Represents a dining session for a single table
    """
    __tablename__ = "table_seatings"

    id = Column(Integer, primary_key=True, autoincrement=True)
    table_id = Column(Integer, ForeignKey("tables.id"), nullable=False, index=True)
    opened_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    closed_at = Column(DateTime(timezone=True), nullable=True)
    assigned_server_id = Column(Integer, ForeignKey("staff.id"), nullable=True)
    notes = Column(Text, nullable=True)

    # relationships
    assigned_server = relationship("Staff", back_populates="table_seatings")
    table = relationship("Table", back_populates="seatings")
    checks = relationship("Check", back_populates="seating")
