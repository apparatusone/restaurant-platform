from sqlalchemy import Column, Integer, String, DateTime, ForeignKey, Text, Boolean
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from shared.dependencies.database import Base

class TableSession(Base):
    """
    Represents a dining session that can span one or multiple tables
    """
    __tablename__ = "table_sessions"

    id = Column(Integer, primary_key=True, autoincrement=True)
    
    # timestamps
    opened_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    closed_at = Column(DateTime(timezone=True), nullable=True)  # NULL while session is active
    
    assigned_server_id = Column(Integer, ForeignKey('staff.id'), nullable=True)
    
    notes = Column(Text, nullable=True)  # Special requests, allergies, etc.
    
    # optional reservation details
    held_until = Column(DateTime(timezone=True), nullable=True)  # For held tables
    is_reservation = Column(Boolean, default=False, nullable=False)  # Mark session as reservation
    customer_name = Column(String(100), nullable=True)  # Customer name for reservations
    

    # relationships
    assigned_server = relationship("Staff", back_populates="table_sessions")
    tables = relationship("Table", foreign_keys="Table.current_session_id", back_populates="current_session")
    checks = relationship("Check", back_populates="session")