from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..controllers import order_items as controller
from shared.schemas import order_items as schema
from shared.dependencies.database import get_db

router = APIRouter(
    tags=['Order Items'],
    prefix="/orderitems"
)


@router.post("/", response_model=schema.OrderItem)
def create(request: schema.OrderItemCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.post("/check/{check_id}", response_model=schema.OrderItem)
def create_for_check(check_id: int, request: schema.CheckItemCreate, db: Session = Depends(get_db)):
    """Create an order item for a check's order"""
    return controller.create_for_check(
        db=db, 
        check_id=check_id,
        menu_item_id=request.menu_item_id,
        quantity=request.quantity,
        special_instructions=request.special_instructions
    )


@router.get("/", response_model=list[schema.OrderItem])
def read_all(status: str = None, check_id: int = None, db: Session = Depends(get_db)):
    return controller.read_all(db, status=status, check_id=check_id)


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