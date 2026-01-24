from sqlalchemy.orm import Session, joinedload
from fastapi import HTTPException, status
from datetime import datetime, timezone
from shared.repositories import BaseRepository
from ..models.table_seatings import TableSeating
from ..models.tables import Table
from ..models.staff import Staff
from ..schemas.table_seatings import TableSessionCreate, TableSessionUpdate

# Initialize repository
session_repo = BaseRepository[TableSeating, TableSessionCreate, TableSessionUpdate](TableSeating)


def create(db: Session, request: TableSessionCreate):
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
    
    # check if the server exists
    if request.assigned_server_id:
        server = db.query(Staff).filter(Staff.id == request.assigned_server_id).first()
        if not server:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Server with id {request.assigned_server_id} not found"
            )
    
    return session_repo.create(db, request)


def read_all(db: Session):
    """Get all sessions with server info"""
    return db.query(TableSeating).options(
        joinedload(TableSeating.assigned_server)
    ).all()


def read_active_sessions(db: Session):
    """Get only active (unclosed) sessions"""
    return db.query(TableSeating).filter(
        TableSeating.closed_at.is_(None)
    ).options(
        joinedload(TableSeating.assigned_server)
    ).all()


def read_one(db: Session, seating_id: int):
    """Get single session with server info"""
    session = db.query(TableSeating).options(
        joinedload(TableSeating.assigned_server)
    ).filter(TableSeating.id == session_id).first()
    
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session with id {session_id} not found"
        )
    return session


def update(db: Session, seating_id: int, request: TableSessionUpdate):
    """Update session with server validation"""
    session = session_repo.get_or_404(db, session_id)
    
    # Validate server exists if being assigned
    if request.assigned_server_id:
        server = db.query(Staff).filter(Staff.id == request.assigned_server_id).first()
        if not server:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Server with id {request.assigned_server_id} not found"
            )
    
    return session_repo.update(db, session_id, request)


def close_session(db: Session, seating_id: int):
    """Close a session and free up the table"""
    session = session_repo.get_or_404(db, session_id)
    
    if session.closed_at:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Session is already closed"
        )
    
    # Check if there are any open checks for this session
    from ..models.checks import Check
    open_checks = db.query(Check).filter(
        Check.seating_id == session_id,
        Check.status != 'closed'
    ).all()
    
    if open_checks:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot close session: {len(open_checks)} open check(s) must be closed first"
        )
    
    session.closed_at = datetime.now(timezone.utc)
    db.commit()
    db.refresh(session)
    return session
