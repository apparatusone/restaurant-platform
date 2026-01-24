# Restaurant Service Schemas

from .table import (
    TableBase,
    TableCreate,
    TableUpdate,
    Table,
    TableWithSession,
    TableCode,
    SectionStr,
    NotesStr as TableNotesStr
)

from .table_seating import (
    TableSeatingBase,
    TableSeatingCreate,
    TableSeatingUpdate,
    TableSeating,
    NotesStr as SeatingNotesStr
)

__all__ = [
    # Table schemas
    "TableBase",
    "TableCreate",
    "TableUpdate",
    "Table",
    "TableWithSession",
    "TableCode",
    "SectionStr",
    "TableNotesStr",
    # Table seating schemas
    "TableSeatingBase",
    "TableSeatingCreate",
    "TableSeatingUpdate",
    "TableSeating",
    "SeatingNotesStr",
]
