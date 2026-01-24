from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from shared.dependencies.database import get_db
from ..schemas import table_seating as schema
from ..controllers import table_seating as controller

router = APIRouter(prefix="/seatings", tags=["seatings"])


@router.post("/", response_model=schema.TableSeating)
def create_seating(request: schema.TableSeatingCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.TableSeating])
def get_all_seatings(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/active", response_model=list[schema.TableSeating])
def get_active_seatings(db: Session = Depends(get_db)):
    return controller.read_active_sessions(db)


@router.get("/{seating_id}", response_model=schema.TableSeating)
def get_seating(seating_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, seating_id=seating_id)


@router.put("/{seating_id}", response_model=schema.TableSeating)
def update_seating(seating_id: int, request: schema.TableSeatingUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, seating_id=seating_id, request=request)


@router.post("/{seating_id}/close", response_model=schema.TableSeating)
def close_seating(seating_id: int, db: Session = Depends(get_db)):
    return controller.close_session(db=db, seating_id=seating_id)
