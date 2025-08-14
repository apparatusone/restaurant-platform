from pydantic import BaseModel, Field, ConfigDict, NonNegativeInt
from pydantic import StringConstraints
from typing import Annotated
from datetime import datetime
from typing import Optional
from enum import Enum
from nanoid import generate

def generate_staff_id():
    return generate('0123456789', 4)

# 4 numeric digits for staff_id at the schema layer
StaffID = Annotated[str, StringConstraints(pattern=r"^\d{4}$", min_length=4, max_length=4)]

# strip whitespace and require non-empty names
NameStr = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1)]

# non-negative integer type for counters
FailedAttempts = NonNegativeInt

class StaffRole(str, Enum):
    host = "host"
    server = "server"
    manager = "manager"
    kitchen = "kitchen"
    admin = "admin"


class StaffBase(BaseModel):
    name: NameStr = Field(..., description="Display name")
    role: StaffRole
    is_active: bool = True
    model_config = ConfigDict(str_strip_whitespace=True)


class StaffCreate(StaffBase):
    staff_id: StaffID = Field(
        default_factory=generate_staff_id,
        description="Login ID for staff"
    )
    pin: str = Field(..., min_length=4, max_length=6, description="Plaintext PIN; will be hashed server-side")
    failed_attempts: FailedAttempts = 0 # non-negative attempt counter
    last_login: Optional[datetime] = None


class StaffUpdate(BaseModel):
    # staff_id is excluded to keep immutable
    # delete and create new staff if id "compromised"
    name: Optional[NameStr] = None
    role: Optional[StaffRole] = None
    pin: Optional[str] = Field(None, min_length=4, max_length=6, description="New PIN; hashed server-side")
    is_active: Optional[bool] = None
    failed_attempts: Optional[FailedAttempts] = None
    last_login: Optional[datetime] = None


class Staff(StaffBase):
    # Expose DB PK as `id` to API consumers while keeping the internal field name explicit
    id: int
    staff_id: StaffID = Field(
        default_factory=generate_staff_id,
        description="Login ID for staff"
    )
    failed_attempts: FailedAttempts
    last_login: Optional[datetime] = None
    model_config = ConfigDict(from_attributes=True)