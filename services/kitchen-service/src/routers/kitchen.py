from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from shared.dependencies.database import get_db
from ..controllers import kitchen as controller
from ..controllers import checks as check_controller
from shared.schemas import orders as order_schema
from shared.schemas import order_items as item_schema
from pydantic import BaseModel

router = APIRouter(prefix="/kitchen", tags=["kitchen"])


@router.get("/queue", response_model=list[order_schema.Order])
def get_kitchen_queue(db: Session = Depends(get_db)):
    """Get all orders currently in kitchen"""
    return controller.get_kitchen_queue(db)


@router.get("/items", response_model=list[item_schema.OrderItem])
def get_kitchen_items(db: Session = Depends(get_db)):
    """Get all order items in prep (SENT status)"""
    return controller.get_kitchen_items(db)


class MarkReadyRequest(BaseModel):
    item_id: int


@router.put("/items/{item_id}/ready", response_model=item_schema.OrderItem)
def mark_item_ready(item_id: int, db: Session = Depends(get_db)):
    """Mark an order item as ready"""
    try:
        return controller.mark_item_ready(db, item_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


class BulkMarkReadyRequest(BaseModel):
    item_ids: list[int]


@router.put("/items/ready/bulk", response_model=list[item_schema.OrderItem])
def mark_items_ready_bulk(request: BulkMarkReadyRequest, db: Session = Depends(get_db)):
    """Mark multiple order items as ready"""
    try:
        return controller.mark_items_ready_bulk(db, request.item_ids)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/checks/{check_id}/send")
def send_check_to_kitchen(check_id: int, db: Session = Depends(get_db)):
    """Send all unsent items in a check to kitchen"""
    try:
        return check_controller.send_check_to_kitchen(db, check_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
