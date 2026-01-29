from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..controllers import check_items as controller
from shared.schemas import check_items as schema
from shared.dependencies.database import get_db

router = APIRouter(
    tags=['Check Items'],
    prefix="/check-items"
)


@router.post("/", response_model=schema.CheckItem)
def create(request: schema.CheckItemCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.post("/check/{check_id}", response_model=schema.CheckItem)
def create_for_check(check_id: int, request: schema.CheckItemCreate, db: Session = Depends(get_db)):
    """Create a check item for a check"""
    return controller.create_for_check(
        db=db, 
        check_id=check_id,
        menu_item_id=request.menu_item_id,
        quantity=request.quantity,
        special_instructions=request.special_instructions
    )


@router.get("/", response_model=list[schema.CheckItem])
def read_all(status: str = None, check_id: int = None, db: Session = Depends(get_db)):
    return controller.read_all(db, status=status, check_id=check_id)


@router.get("/{item_id}", response_model=schema.CheckItem)
def read_one(item_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, item_id=item_id)


@router.put("/{item_id}", response_model=schema.CheckItem)
def update(item_id: int, request: schema.CheckItemUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, request=request, item_id=item_id)


@router.delete("/{item_id}")
def delete(item_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, item_id=item_id)


@router.put("/{item_id}/ready", response_model=schema.CheckItem)
def mark_item_ready(item_id: int, db: Session = Depends(get_db)):
    return controller.mark_item_ready(db=db, item_id=item_id)
