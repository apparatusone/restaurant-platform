from sqlalchemy import Column, Integer, DateTime, text
from sqlalchemy.sql import func
from shared.dependencies.database import Base


class TimeClockEntry(Base):
    __tablename__ = "timeclock_entries"

    id = Column(Integer, primary_key=True, autoincrement=True)
    staff_id = Column(Integer, nullable=False, index=True)
    clock_in_at = Column(DateTime, nullable=False, server_default=func.now())
    clock_out_at = Column(DateTime, nullable=True)
    created_at = Column(DateTime, nullable=False, server_default=func.now())
