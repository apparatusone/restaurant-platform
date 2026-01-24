from pydantic import BaseModel, Field, ConfigDict, StringConstraints
from typing import Annotated, Optional
from datetime import datetime


NotesStr = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=500)]


class TableSeatingBase(BaseModel):
    assigned_server_id: Optional[int] = Field(None, description="FK -> staff.id")
    notes: Optional[NotesStr] = Field(None, description="Special requests, allergies, etc")

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class TableSeatingCreate(TableSeatingBase):
    table_id: int = Field(..., description="Table ID for this seating")


class TableSeatingUpdate(BaseModel):
    assigned_server_id: Optional[int] = None
    notes: Optional[NotesStr] = None

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class TableSeating(TableSeatingBase):
    id: int
    table_id: int
    opened_at: datetime
    closed_at: Optional[datetime] = None
    
    model_config = ConfigDict(from_attributes=True)
