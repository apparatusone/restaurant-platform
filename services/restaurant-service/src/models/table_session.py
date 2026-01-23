from sqlalchemy import Column, Integer, DateTime, Text, ForeignKey
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from shared.dependencies.database import Base

class TableSession(Base):
    """
    Represents a dining session for a single table
    """
    __tablename__ = "table_sessions"

    id = Column(Integer, primary_key=True, autoincrement=True)
    table_id = Column(Integer, nullable=False, index=True)
    opened_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    closed_at = Column(DateTime(timezone=True), nullable=True)
    assigned_server_id = Column(Integer, ForeignKey("staff.id"), nullable=True)
    notes = Column(Text, nullable=True)

    # relationships
    assigned_server = relationship("Staff", back_populates="table_sessions")
