from fastapi import APIRouter, Depends, Header, HTTPException
from sqlalchemy.orm import Session

from shared.dependencies.database import get_db
from ..controllers import timeclock as controller
from ..schemas.timeclock import (
    TimeclockStatusResponse,
    TimeclockClockInResponse,
    TimeclockClockOutResponse,
)


router = APIRouter(prefix="/timeclock", tags=["timeclock"])


def _parse_staff_id(x_staff_id: str | None) -> int:
    if not x_staff_id:
        raise HTTPException(status_code=401, detail="Missing X-Staff-Id header")
    try:
        return int(x_staff_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid X-Staff-Id header")


@router.get("/status", response_model=TimeclockStatusResponse, status_code=200)
def status(db: Session = Depends(get_db), x_staff_id: str | None = Header(default=None, alias="X-Staff-Id")):
    staff_id = _parse_staff_id(x_staff_id)
    return controller.get_status(db, staff_id)


@router.post("/clock-in", response_model=TimeclockClockInResponse, status_code=200)
def clock_in(db: Session = Depends(get_db), x_staff_id: str | None = Header(default=None, alias="X-Staff-Id")):
    staff_id = _parse_staff_id(x_staff_id)
    return controller.clock_in(db, staff_id)


@router.post("/clock-out", response_model=TimeclockClockOutResponse, status_code=200)
def clock_out(db: Session = Depends(get_db), x_staff_id: str | None = Header(default=None, alias="X-Staff-Id")):
    staff_id = _parse_staff_id(x_staff_id)
    return controller.clock_out(db, staff_id)
