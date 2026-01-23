from sqlalchemy.orm import Session, joinedload
from fastapi import HTTPException, status
from datetime import datetime, timezone
from shared.repositories import BaseRepository
from ..models.table_session import TableSession
from ..models.table import Table
from ..schemas.table_session import TableSessionCreate, TableSessionUpdate

# Initialize repository
session_repo = BaseRepository[TableSession, TableSessionCreate, TableSessionUpdate](TableSession)


def create(db: Session, request: TableSessionCreate):
    """Create table session"""
    # Validate table exists
    table = db.query(Table).filter(Table.id == request.table_id).first()
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table not found: {request.table_id}"
        )
    
    # Check if table is already occupied
    active_session = db.query(TableSession).filter(
        TableSession.table_id == request.table_id,
        TableSession.closed_at.is_(None)
    ).first()
    
    if active_session:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table {table.code} is already occupied"
        )
    
    # Note: Server validation will be handled via staff-service in the future
    # For now, we accept assigned_server_id without validation
    
    return session_repo.create(db, request)


def read_all(db: Session):
    """Get all sessions"""
    return db.query(TableSession).all()


def read_active_sessions(db: Session):
    """Get only active (unclosed) sessions"""
    return db.query(TableSession).filter(
        TableSession.closed_at.is_(None)
    ).all()


def read_one(db: Session, session_id: int):
    """Get single session"""
    session = db.query(TableSession).filter(TableSession.id == session_id).first()
    
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session with id {session_id} not found"
        )
    return session


def update(db: Session, session_id: int, request: TableSessionUpdate):
    """Update session"""
    session = session_repo.get_or_404(db, session_id)
    
    # Note: Server validation will be handled via staff-service in the future
    # For now, we accept assigned_server_id without validation
    
    return session_repo.update(db, session_id, request)


def close_session(db: Session, session_id: int):
    """Close a session and free up the table"""
    session = session_repo.get_or_404(db, session_id)
    
    if session.closed_at:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Session is already closed"
        )
    
    # Note: Check validation will be handled via order-service in the future
    # For now, we allow closing sessions without check validation
    
    session.closed_at = datetime.now(timezone.utc)
    db.commit()
    db.refresh(session)
    return session
