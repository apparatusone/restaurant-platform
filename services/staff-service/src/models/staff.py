from sqlalchemy import CHAR, Column, Integer, String, Boolean, DateTime, CheckConstraint, text
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from shared.dependencies.database import Base

class Staff(Base):
    __tablename__ = "staff"
    id = Column(Integer, primary_key=True, autoincrement=True)
    staff_id = Column(CHAR(4), unique=True, nullable=False)
    name = Column(String(50), nullable=False)
    role = Column(String(16), nullable=False)
    is_active = Column(Boolean, nullable=False, server_default=text("1"))
    failed_attempts = Column(Integer, nullable=False, server_default=text("0"))
    pin_hash = Column(String(128), nullable=False)
    last_login = Column(DateTime, nullable=True)
    created_at = Column(DateTime, nullable=False, server_default=func.now())

    # relationships
    table_sessions = relationship("TableSession", back_populates="assigned_server")

    __table_args__ = (
        CheckConstraint("staff_id REGEXP '^[0-9]{4}$'", name="staff_id_numeric_check"),
    )
