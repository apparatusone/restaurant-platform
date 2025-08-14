from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..controllers import orders as controller
from ..schemas import orders as schema

from ..dependencies.database import get_db

router = APIRouter(prefix="/orders", tags=["orders"])


@router.post("/", response_model=schema.Order)
def create_order(request: schema.OrderCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.Order])
def get_all_orders(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/kitchen", response_model=list[schema.Order])
def get_kitchen_orders(db: Session = Depends(get_db)):
    return controller.get_kitchen_orders(db)


@router.get("/delivery", response_model=list[schema.Order])
def get_delivery_orders(db: Session = Depends(get_db)):
    return controller.get_delivery_orders(db)


@router.get("/pickup", response_model=list[schema.Order])
def get_pickup_orders(db: Session = Depends(get_db)):
    return controller.get_pickup_orders(db)


@router.get("/{order_id}", response_model=schema.Order)
def get_order(order_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, order_id=order_id)


@router.put("/{order_id}", response_model=schema.Order)
def update_order(order_id: int, request: schema.OrderUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, order_id=order_id, request=request)


@router.delete("/{order_id}")
def delete_order(order_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, order_id=order_id)



