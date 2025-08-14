from sqlalchemy import Column, Integer, String, Boolean, DateTime, ForeignKey, Text
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..dependencies.database import Base

class Table(Base):
    """
    Represents physical tables in the restaurant
    These are essentially persistent
    """
    __tablename__ = "tables"

    id = Column(Integer, primary_key=True, autoincrement=True)
    code = Column(String(8), unique=True, nullable=False, index=True)
    capacity = Column(Integer, nullable=False)
    section = Column(String(32), nullable=True)
    is_outdoor = Column(Boolean, default=False, nullable=False)
    notes = Column(Text, nullable=True)
    
    current_session_id = Column(Integer, ForeignKey('table_sessions.id'), nullable=True)
    
    # timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())


    # relationships
    current_session = relationship("TableSession", foreign_keys=[current_session_id], back_populates="tables")