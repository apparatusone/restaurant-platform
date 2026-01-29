from datetime import datetime
from pydantic import BaseModel


class TimeclockStatusResponse(BaseModel):
    clocked_in: bool
    clock_in_at: datetime | None = None


class TimeclockClockInResponse(BaseModel):
    clocked_in: bool
    clock_in_at: datetime


class TimeclockClockOutResponse(BaseModel):
    clocked_in: bool
    clock_in_at: datetime | None = None
    clock_out_at: datetime | None = None
