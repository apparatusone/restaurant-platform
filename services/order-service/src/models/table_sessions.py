from sqlalchemy import Column, Integer, String, DateTime, ForeignKey, Text
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from shared.dependencies.database import Base

class TableSession(Base):
    """
    Represents a dining session for a single table
    """
    __tablename__ = "table_sessions"

    id = Column(Integer, primary_key=True, autoincrement=True)
    
    # single table per session
    table_id = Column(Integer, ForeignKey('tables.id', ondelete='CASCADE'), nullable=False)
    
    # timestamps
    opened_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    closed_at = Column(DateTime(timezone=True), nullable=True)  # NULL while session is active
    
    assigned_server_id = Column(Integer, ForeignKey('staff.id'), nullable=True)
    
    notes = Column(Text, nullable=True)  # Special requests, allergies, etc.

    # relationships
    assigned_server = relationship("Staff", back_populates="table_sessions")
    table = relationship("Table", back_populates="sessions")
    checks = relationship("Check", back_populates="session")