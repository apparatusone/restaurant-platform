from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from shared.dependencies.database import get_db
from ..schemas import table_session as schema
from ..controllers import table_session as controller

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


@router.get("/{session_id}", response_model=schema.TableSession)
def get_session(session_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, session_id=session_id)


@router.put("/{session_id}", response_model=schema.TableSessionUpdate)
def update_session(session_id: int, request: schema.TableSessionUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, session_id=session_id, request=request)


@router.post("/{session_id}/close", response_model=schema.TableSession)
def close_session(session_id: int, db: Session = Depends(get_db)):
    return controller.close_session(db=db, session_id=session_id)
