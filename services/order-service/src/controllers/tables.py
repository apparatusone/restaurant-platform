from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from ..models.tables import Table
from ..models.table_sessions import TableSession
from ..schemas.tables import TableCreate, TableUpdate
from ..utils.errors import (
    raise_not_found,
    raise_validation_error,
    raise_conflict_error
)


def create(db: Session, request: TableCreate):
    existing_table = db.query(Table).filter(Table.code == request.code).first()
    if existing_table:
        raise_conflict_error(f"Table with code '{request.code}' already exists")
    
    new_table = Table(
        code=request.code,
        capacity=request.capacity,
        section=request.section,
        is_outdoor=request.is_outdoor,
        notes=request.notes
    )
    
    db.add(new_table)
    db.commit()
    db.refresh(new_table)
    return new_table


def read_all(db: Session):
    return db.query(Table).all()


def read_one(db: Session, table_id: int):
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise_not_found("Table", table_id)
    return table


def read_by_code(db: Session, table_code: str):
    table = db.query(Table).filter(Table.code == table_code).first()
    if not table:
        raise_not_found("Table", table_code)
    return table


def update(db: Session, table_id: int, request: TableUpdate):
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise_not_found("Table", table_id)
    
    # check for code conflict
    if request.code and request.code != table.code:
        existing_table = db.query(Table).filter(Table.code == request.code).first()
        if existing_table:
            raise_conflict_error(f"Table with code '{request.code}' already exists")
    
    update_data = request.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(table, field, value)
    
    db.commit()
    db.refresh(table)
    return table


def delete(db: Session, table_id: int):
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise_not_found("Table", table_id)
    
    # Check if table has an active session
    active_session = db.query(TableSession).filter(
        TableSession.table_id == table_id,
        TableSession.closed_at.is_(None)
    ).first()
    
    if active_session:
        raise_validation_error("Cannot delete table with active session")
    
    db.delete(table)
    db.commit()
    return {"message": f"Table {table.code} deleted successfully"}


def get_available_tables(db: Session):
    """Get tables that don't have an active (unclosed) session"""
    occupied_table_ids = db.query(TableSession.table_id).filter(
        TableSession.closed_at.is_(None)
    ).subquery()
    
    return db.query(Table).filter(
        Table.id.notin_(occupied_table_ids)
    ).all()


def get_occupied_tables(db: Session):
    """Get tables that have an active (unclosed) session"""
    return db.query(Table).join(
        TableSession,
        Table.id == TableSession.table_id
    ).filter(
        TableSession.closed_at.is_(None)
    ).all()


def get_tables_with_session_info(db: Session):
    """Get all tables with their current session ID (for floor view) - single query with LEFT JOIN"""
    from sqlalchemy import and_
    
    results = db.query(
        Table,
        TableSession.id.label('current_session_id')
    ).outerjoin(
        TableSession,
        and_(
            TableSession.table_id == Table.id,
            TableSession.closed_at.is_(None)
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
            "current_session_id": session_id
        }
        for table, session_id in results
    ]