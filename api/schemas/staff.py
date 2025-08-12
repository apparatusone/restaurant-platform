from pydantic import BaseModel, Field
from typing import Optional
from enum import Enum
from nanoid import generate

def generate_staff_id():
    return generate('0123456789', 4)

class StaffRole(str, Enum):
    host = "host"
    waiter = "waiter"
    manager = "manager"
    kitchen = "kitchen"

class StaffBase(BaseModel):
    name: str = Field(..., description="Display name")
    role: StaffRole
    is_active: bool = True

class StaffCreate(StaffBase):
    staff_id: str = Field(
        default_factory=generate_staff_id,
        description="Login ID for staff"
    )
    pin_hash: str
    pin_salt: str
    failed_attempts: int = 0
    last_login: Optional[str] = None  # ISO format

class StaffUpdate(BaseModel):
    name: Optional[str] = None
    role: Optional[StaffRole] = None
    is_active: Optional[bool] = None
    staff_id: Optional[str] = None
    pin_hash: Optional[str] = None
    pin_salt: Optional[str] = None
    failed_attempts: Optional[int] = None
    last_login: Optional[str] = None

class Staff(StaffBase):
    db_id: int = Field(..., description="Internal DB primary key", alias="id")
    staff_id: str = Field(
        default_factory=generate_staff_id,
        description="Login ID for staff"
    )
    pin_hash: str
    pin_salt: str
    failed_attempts: int
    last_login: Optional[str] = None

    class Config:
        orm_mode = True
        allow_population_by_field_name = True