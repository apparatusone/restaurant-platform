from datetime import datetime
from fastapi import HTTPException, status
from sqlalchemy.orm import Session
from sqlalchemy.sql import func

from ..models.timeclock import TimeClockEntry


def _get_active_entry(db: Session, staff_id: int) -> TimeClockEntry | None:
    return (
        db.query(TimeClockEntry)
        .filter(TimeClockEntry.staff_id == staff_id, TimeClockEntry.clock_out_at.is_(None))
        .order_by(TimeClockEntry.clock_in_at.desc())
        .first()
    )


def get_status(db: Session, staff_id: int):
    entry = _get_active_entry(db, staff_id)
    if not entry:
        return {"clocked_in": False, "clock_in_at": None}
    return {"clocked_in": True, "clock_in_at": entry.clock_in_at}


def clock_in(db: Session, staff_id: int):
    entry = _get_active_entry(db, staff_id)
    if entry:
        return {"clocked_in": True, "clock_in_at": entry.clock_in_at}

    new_entry = TimeClockEntry(staff_id=staff_id)
    db.add(new_entry)
    db.commit()
    db.refresh(new_entry)
    return {"clocked_in": True, "clock_in_at": new_entry.clock_in_at}


def clock_out(db: Session, staff_id: int):
    entry = _get_active_entry(db, staff_id)
    if not entry:
        return {"clocked_in": False, "clock_in_at": None, "clock_out_at": None}

    entry.clock_out_at = datetime.now()
    db.commit()
    db.refresh(entry)

    return {"clocked_in": False, "clock_in_at": entry.clock_in_at, "clock_out_at": entry.clock_out_at}
