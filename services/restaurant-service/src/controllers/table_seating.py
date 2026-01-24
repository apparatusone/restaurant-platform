from sqlalchemy.orm import Session, joinedload
from fastapi import HTTPException, status
from datetime import datetime, timezone
from shared.repositories import BaseRepository
from ..models.table_seating import TableSeating
from ..models.table import Table
from ..schemas.table_seating import TableSeatingCreate, TableSeatingUpdate

# Initialize repository
seating_repo = BaseRepository[TableSeating, TableSeatingCreate, TableSeatingUpdate](TableSeating)


def create(db: Session, request: TableSeatingCreate):
    """Create table seating"""
    # Validate table exists
    table = db.query(Table).filter(Table.id == request.table_id).first()
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table not found: {request.table_id}"
        )
    
    # Check if table is already occupied
    active_session = db.query(TableSeating).filter(
        TableSeating.table_id == request.table_id,
        TableSeating.closed_at.is_(None)
    ).first()
    
    if active_session:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table {table.code} is already occupied"
        )
    
    # Note: Server validation will be handled via staff-service in the future
    # For now, we accept assigned_server_id without validation
    
    return seating_repo.create(db, request)


def read_all(db: Session):
    """Get all sessions"""
    return db.query(TableSeating).all()


def read_active_sessions(db: Session):
    """Get only active (unclosed) sessions"""
    return db.query(TableSeating).filter(
        TableSeating.closed_at.is_(None)
    ).all()


def read_one(db: Session, seating_id: int):
    """Get single seating"""
    seating = db.query(TableSeating).filter(TableSeating.id == seating_id).first()
    
    if not seating:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Seating with id {seating_id} not found"
        )
    return seating


def update(db: Session, seating_id: int, request: TableSeatingUpdate):
    """Update seating"""
    seating = seating_repo.get_or_404(db, seating_id)
    
    # Note: Server validation will be handled via staff-service in the future
    # For now, we accept assigned_server_id without validation
    
    return seating_repo.update(db, seating_id, request)


def close_session(db: Session, seating_id: int):
    """Close a seating and free up the table"""
    seating = seating_repo.get_or_404(db, seating_id)
    
    if seating.closed_at:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Seating is already closed"
        )
    
    # Note: Check validation will be handled via order-service in the future
    # For now, we allow closing seatings without check validation
    
    seating.closed_at = datetime.now(timezone.utc)
    db.commit()
    db.refresh(seating)
    return seating
