from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from ..models.tables import Table
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
    
    if table.current_session_id:
        raise_validation_error("Cannot delete table with active session")
    
    db.delete(table)
    db.commit()
    return {"message": f"Table {table.code} deleted successfully"}


def get_available_tables(db: Session):
    return db.query(Table).filter(Table.current_session_id.is_(None)).all()


def get_occupied_tables(db: Session):
    return db.query(Table).filter(Table.current_session_id.isnot(None)).all()