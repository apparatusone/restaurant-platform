from pydantic import BaseModel, Field, ConfigDict, PositiveInt, StringConstraints
from typing import Annotated, Optional


TableCode = Annotated[str, StringConstraints(pattern=r"^[A-Za-z0-9_-]{1,8}$", strip_whitespace=True, min_length=1)]
SectionStr = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=32)]
NotesStr = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=280)]


class TableBase(BaseModel):
    code: TableCode = Field(..., description="'A1' or '12', etc")
    capacity: PositiveInt = Field(..., le=20, description="Maximum number of people the table can hold")
    section: Optional[SectionStr] = Field(None, description="Zone/area label")
    is_outdoor: bool = False
    notes: Optional[NotesStr] = None

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class TableCreate(TableBase):
    pass


class TableUpdate(BaseModel):
    code: Optional[TableCode] = None
    capacity: Optional[PositiveInt] = Field(None, le=20)
    section: Optional[SectionStr] = None
    is_outdoor: Optional[bool] = None
    notes: Optional[NotesStr] = None

    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")


class Table(TableBase):
    id: int
    
    model_config = ConfigDict(from_attributes=True)


class TableWithSession(Table):
    """Table with current session info for floor view"""
    current_seating_id: Optional[int] = None
