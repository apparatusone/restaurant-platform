from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..controllers import staff as controller
from ..schemas import staff as schema
from shared.dependencies.database import get_db

router = APIRouter(
    tags=['Staff'],
    prefix="/staff"
)


@router.post("/", response_model=schema.Staff)
def create(request: schema.StaffCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.Staff])
def read_all(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/{id}", response_model=schema.Staff)
def read_one(id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, id=id)


@router.put("/{id}", response_model=schema.Staff)
def update(id: int, request: schema.StaffUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, request=request, id=id)


@router.delete("/{id}")
def delete(id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, id=id)
