from sqlalchemy.orm import Session
from sqlalchemy import and_
from shared.repositories import BaseRepository
from ..models.table import Table
from ..models.table_seating import TableSeating
from ..schemas.table import TableCreate, TableUpdate
from fastapi import HTTPException, status

# Initialize repository
table_repo = BaseRepository[Table, TableCreate, TableUpdate](Table)


def create(db: Session, request: TableCreate):
    # Check for duplicate code before creating
    existing_table = table_repo.get_by_field(db, "code", request.code)
    if existing_table:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"Table with code '{request.code}' already exists"
        )
    
    return table_repo.create(db, request)


def read_all(db: Session):
    return table_repo.get_all(db)


def read_one(db: Session, table_id: int):
    return table_repo.get_or_404(db, table_id)


def read_by_code(db: Session, table_code: str):
    return table_repo.get_by_field_or_404(db, "code", table_code)


def update(db: Session, table_id: int, request: TableUpdate):
    table = table_repo.get_or_404(db, table_id)
    
    # check for code conflict
    if request.code and request.code != table.code:
        existing_table = table_repo.get_by_field(db, "code", request.code)
        if existing_table:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=f"Table with code '{request.code}' already exists"
            )
    
    return table_repo.update(db, table_id, request)


def delete(db: Session, table_id: int):
    # Check if table has an active session
    active_session = db.query(TableSeating).filter(
        TableSeating.table_id == table_id,
        TableSeating.closed_at.is_(None)
    ).first()
    
    if active_session:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot delete table with active session"
        )
    
    table_repo.delete(db, table_id)
    
    # Return success message
    table = table_repo.get(db, table_id)
    code = table.code if table else table_id
    return {"message": f"Table {code} deleted successfully"}


def get_available_tables(db: Session):
    """Get tables that don't have an active (unclosed) session"""
    from sqlalchemy import select
    
    occupied_table_ids = select(TableSeating.table_id).filter(
        TableSeating.closed_at.is_(None)
    ).scalar_subquery()
    
    return db.query(Table).filter(
        Table.id.notin_(occupied_table_ids)
    ).all()


def get_occupied_tables(db: Session):
    """Get tables that have an active (unclosed) session"""
    return db.query(Table).join(
        TableSeating,
        Table.id == TableSeating.table_id
    ).filter(
        TableSeating.closed_at.is_(None)
    ).all()


def get_tables_with_session_info(db: Session):
    """Get all tables with their current session ID (for floor view) - single query with LEFT JOIN"""
    results = db.query(
        Table,
        TableSeating.id.label('current_seating_id')
    ).outerjoin(
        TableSeating,
        and_(
            TableSeating.table_id == Table.id,
            TableSeating.closed_at.is_(None)
        )
    ).all()
    
    return [
        {
            "id": table.id,
            "code": table.code,
            "capacity": table.capacity,
            "section": table.section,
            "is_outdoor": table.is_outdoor,
            "notes": table.notes,
            "current_seating_id": seating_id
        }
        for table, seating_id in results
    ]
