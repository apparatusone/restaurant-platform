from pydantic import BaseModel, Field, ConfigDict, StringConstraints
from typing import Annotated, Optional
from datetime import datetime


NotesStr = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=500)]


class TableSessionBase(BaseModel):
    assigned_server_id: Optional[int] = Field(None, description="FK -> staff.id")
    notes: Optional[NotesStr] = Field(None, description="Special requests, allergies, etc")

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class TableSessionCreate(TableSessionBase):
    table_id: int = Field(..., description="Table ID for this session")


class TableSessionUpdate(BaseModel):
    assigned_server_id: Optional[int] = None
    notes: Optional[NotesStr] = None

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class TableSession(TableSessionBase):
    id: int
    table_id: int
    opened_at: datetime
    closed_at: Optional[datetime] = None
    
    model_config = ConfigDict(from_attributes=True)
