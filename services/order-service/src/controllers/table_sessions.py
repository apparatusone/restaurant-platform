from sqlalchemy.orm import Session, joinedload
from fastapi import HTTPException, status
from datetime import datetime
from ..models.table_sessions import TableSession
from ..models.tables import Table
from ..models.staff import Staff
from ..schemas.table_sessions import TableSessionCreate, TableSessionUpdate


def create(db: Session, request: TableSessionCreate):
    table = db.query(Table).filter(Table.id == request.table_id).first()
    
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table not found: {request.table_id}"
        )
    
    # check if table is already occupied (has an active session)
    active_session = db.query(TableSession).filter(
        TableSession.table_id == request.table_id,
        TableSession.closed_at.is_(None)
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
    
    new_session = TableSession(
        table_id=request.table_id,
        assigned_server_id=request.assigned_server_id,
        notes=request.notes
    )
    
    db.add(new_session)
    db.commit()
    db.refresh(new_session)
    return new_session


def read_all(db: Session):
    return db.query(TableSession).options(joinedload(TableSession.assigned_server)).all()


def read_active_sessions(db: Session):
    return db.query(TableSession).filter(TableSession.closed_at.is_(None)).options(
        joinedload(TableSession.assigned_server)
    ).all()





def read_one(db: Session, session_id: int):
    session = db.query(TableSession).options(
        joinedload(TableSession.assigned_server)
    ).filter(TableSession.id == session_id).first()
    
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session with id {session_id} not found"
        )
    return session


def update(db: Session, session_id: int, request: TableSessionUpdate):
    session = db.query(TableSession).filter(TableSession.id == session_id).first()
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session with id {session_id} not found"
        )
    
    if request.assigned_server_id:
        server = db.query(Staff).filter(Staff.id == request.assigned_server_id).first()
        if not server:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Server with id {request.assigned_server_id} not found"
            )
    
    update_data = request.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(session, field, value)
    
    db.commit()
    db.refresh(session)
    return session


def close_session(db: Session, session_id: int):
    """Close a session and free up the table"""
    session = db.query(TableSession).filter(TableSession.id == session_id).first()
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session with id {session_id} not found"
        )
    
    if session.closed_at:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Session is already closed"
        )
    
    # Check if there are any open checks for this session
    from ..models.checks import Check
    open_checks = db.query(Check).filter(
        Check.session_id == session_id,
        Check.status != 'closed'
    ).all()
    
    if open_checks:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot close session: {len(open_checks)} open check(s) must be closed first"
        )
    
    session.closed_at = datetime.utcnow()
    
    db.commit()
    db.refresh(session)
    return session


