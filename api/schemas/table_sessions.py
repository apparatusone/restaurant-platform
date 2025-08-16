from pydantic import BaseModel, Field, ConfigDict, PositiveInt, StringConstraints
from typing import Annotated, Optional, List
from datetime import datetime


NotesStr = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=500)]


class TableSessionBase(BaseModel):
    assigned_server_id: Optional[int] = Field(None, description="FK -> staff.id")
    notes: Optional[NotesStr] = Field(None, description="Special requests, allergies, etc")
    held_until: Optional[datetime] = Field(None, description="Hold table until this time")
    is_reservation: bool = Field(False, description="Mark session as reservation")
    customer_name: Optional[str] = Field(None, description="Customer name for reservations")

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class TableSessionCreate(TableSessionBase):
    table_ids: List[int] = Field(..., min_length=1, description="List of table IDs for this session")


class TableSessionUpdate(BaseModel):
    assigned_server_id: Optional[int] = None
    notes: Optional[NotesStr] = None
    held_until: Optional[datetime] = None
    is_reservation: Optional[bool] = None
    customer_name: Optional[str] = None
    closed_at: Optional[datetime] = None  # To close a session

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class TableSession(TableSessionBase):
    id: int
    opened_at: datetime
    closed_at: Optional[datetime] = None
    
    model_config = ConfigDict(from_attributes=True)


class TableSessionWithDetails(TableSession):
    """Extended version with related data"""
    table_codes: List[str] = Field(default_factory=list, description="Codes of tables in this session")
    check_count: int = Field(0, description="Number of checks for this session")
    
    model_config = ConfigDict(from_attributes=True)