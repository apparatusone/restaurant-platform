from sqlalchemy.orm import Session, joinedload
from fastapi import HTTPException, status
from datetime import datetime
from ..models.table_sessions import TableSession
from ..models.tables import Table
from ..models.staff import Staff
from ..schemas.table_sessions import TableSessionCreate, TableSessionUpdate


def create(db: Session, request: TableSessionCreate):
    tables = db.query(Table).filter(Table.id.in_(request.table_ids)).all()
    
    if len(tables) != len(request.table_ids):
        found_ids = [t.id for t in tables]
        missing_ids = [tid for tid in request.table_ids if tid not in found_ids]
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Tables not found: {missing_ids}"
        )
    
    # check if any tables are already occupied
    occupied_tables = [t for t in tables if t.current_session_id is not None]
    if occupied_tables:
        occupied_codes = [t.code for t in occupied_tables]
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Tables already occupied: {occupied_codes}"
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
        assigned_server_id=request.assigned_server_id,
        notes=request.notes,
        held_until=request.held_until,
        is_reservation=request.is_reservation,
        customer_name=request.customer_name
    )
    
    db.add(new_session)
    db.flush()  # Get the session ID
    
    for table in tables:
        table.current_session_id = new_session.id
    
    db.commit()
    db.refresh(new_session)
    return new_session


def read_all(db: Session):
    return db.query(TableSession).options(joinedload(TableSession.assigned_server)).all()


def read_active_sessions(db: Session):
    return db.query(TableSession).filter(TableSession.closed_at.is_(None)).options(
        joinedload(TableSession.assigned_server)
    ).all()


# all reservations
def read_reservation_sessions(db: Session):
    return db.query(TableSession).filter(TableSession.is_reservation == True).options(
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
    """Close a session and free up the tables"""
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
    
    session.closed_at = datetime.utcnow()
    
    tables = db.query(Table).filter(Table.current_session_id == session_id).all()
    for table in tables:
        table.current_session_id = None
    
    db.commit()
    db.refresh(session)
    return session


# merge tables
def add_table_to_session(db: Session, session_id: int, table_id: int):
    session = db.query(TableSession).filter(TableSession.id == session_id).first()
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Session with id {session_id} not found"
        )
    
    if session.closed_at:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot add table to closed session"
        )
    
    table = db.query(Table).filter(Table.id == table_id).first()
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table with id {table_id} not found"
        )
    
    if table.current_session_id:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Table {table.code} is already occupied"
        )
    
    table.current_session_id = session_id
    db.commit()
    
    return {"message": f"Table {table.code} added to session {session_id}"}


def remove_table_from_session(db: Session, session_id: int, table_id: int):
    table = db.query(Table).filter(
        Table.id == table_id,
        Table.current_session_id == session_id
    ).first()
    
    if not table:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Table with id {table_id} not found in session {session_id}"
        )
    
    # Check if this is the last table in the session
    session_tables = db.query(Table).filter(Table.current_session_id == session_id).all()
    if len(session_tables) <= 1:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot remove the last table from a session. Close the session instead."
        )
    
    table.current_session_id = None
    db.commit()
    
    return {"message": f"Table {table.code} removed from session {session_id}"}