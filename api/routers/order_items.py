from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..controllers import order_items as controller
from ..schemas import order_items as schema
from ..dependencies.database import get_db

router = APIRouter(
    tags=['Order Items'],
    prefix="/orderitems"
)


@router.post("/", response_model=schema.OrderItem)
def create(request: schema.OrderItemCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.OrderItem])
def read_all(status: str = None, db: Session = Depends(get_db)):
    return controller.read_all(db, status=status)


@router.get("/{item_id}", response_model=schema.OrderItem)
def read_one(item_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, item_id=item_id)


@router.put("/{item_id}", response_model=schema.OrderItem)
def update(item_id: int, request: schema.OrderItemUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, request=request, item_id=item_id)


@router.delete("/{item_id}")
def delete(item_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, item_id=item_id)


@router.put("/{item_id}/ready", response_model=schema.OrderItem)
def mark_item_ready(item_id: int, db: Session = Depends(get_db)):
    return controller.mark_item_ready(db=db, item_id=item_id)