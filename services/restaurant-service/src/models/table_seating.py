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
    assigned_server_id = Column(Integer, nullable=True)  # FK to staff-service (validated via API)
    notes = Column(Text, nullable=True)

    # relationships
    # Note: Staff relationship removed - staff data lives in staff-service
    # Use assigned_server_id to query staff-service API when needed
    table = relationship("Table", back_populates="seatings")
    # Note: Check relationship removed - checks live in order-service
    # Use seating_id to query order-service API for checks
