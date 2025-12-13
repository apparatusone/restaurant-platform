from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..dependencies.database import get_db
from ..schemas import table_sessions as schema
from ..schemas import checks as check_schema
from ..controllers import table_sessions as controller
from ..controllers import checks as check_controller

router = APIRouter(prefix="/table-sessions", tags=["table-sessions"])


@router.post("/", response_model=schema.TableSession)
def create_session(request: schema.TableSessionCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.TableSession])
def get_all_sessions(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/active", response_model=list[schema.TableSession])
def get_active_sessions(db: Session = Depends(get_db)):
    return controller.read_active_sessions(db)


@router.get("/reservations", response_model=list[schema.TableSession])
def get_reservation_sessions(db: Session = Depends(get_db)):
    return controller.read_reservation_sessions(db)


@router.get("/{session_id}", response_model=schema.TableSession)
def get_session(session_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, session_id=session_id)


@router.put("/{session_id}", response_model=schema.TableSession)
def update_session(session_id: int, request: schema.TableSessionUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, session_id=session_id, request=request)


@router.post("/{session_id}/close", response_model=schema.TableSession)
def close_session(session_id: int, db: Session = Depends(get_db)):
    return controller.close_session(db=db, session_id=session_id)


@router.post("/{session_id}/tables/{table_id}")
def add_table_to_session(session_id: int, table_id: int, db: Session = Depends(get_db)):
    return controller.add_table_to_session(db=db, session_id=session_id, table_id=table_id)


@router.delete("/{session_id}/tables/{table_id}")
def remove_table_from_session(session_id: int, table_id: int, db: Session = Depends(get_db)):
    return controller.remove_table_from_session(db=db, session_id=session_id, table_id=table_id)


# check management endpoints for table sessions
@router.post("/{session_id}/checks", response_model=check_schema.Check)
def create_check_for_session(session_id: int, db: Session = Depends(get_db)):
    """Create a new check for a table session"""
    check_request = check_schema.CheckCreate(
        session_id=session_id,
        is_virtual=False
    )
    return check_controller.create(db=db, request=check_request)


@router.get("/{session_id}/checks", response_model=list[check_schema.Check])
def get_checks_for_session(session_id: int, db: Session = Depends(get_db)):
    """Get all checks for a table session"""
    return check_controller.read_by_session(db=db, session_id=session_id)