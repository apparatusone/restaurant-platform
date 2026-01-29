from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from decimal import Decimal
from pydantic import BaseModel
from typing import Optional
from shared.dependencies.database import get_db
from ..schemas import checks as schema
from shared.schemas import check_items as order_item_schema
from ..controllers import checks as controller
from ..controllers import check_items as order_item_controller

router = APIRouter(prefix="/checks", tags=["checks"])


class CheckPaymentCreate(BaseModel):
    amount: Decimal
    payment_type: str = "cash"
    card_number: Optional[str] = None


@router.post("/", response_model=schema.Check)
async def create_check(request: schema.CheckCreate, db: Session = Depends(get_db)):
    return await controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.Check])
def get_all_checks(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/open", response_model=list[schema.Check])
def get_open_checks(db: Session = Depends(get_db)):
    return controller.get_open_checks(db)


@router.get("/pending", response_model=list[schema.Check])
def get_pending_checks(db: Session = Depends(get_db)):
    return controller.get_pending_checks(db)


@router.get("/{check_id}", response_model=schema.Check)
def get_check(check_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, check_id=check_id)


@router.get("/seating/{seating_id}", response_model=list[schema.Check])
def get_checks_by_seating(seating_id: int, db: Session = Depends(get_db)):
    return controller.read_by_seating(db, seating_id=seating_id)


@router.put("/{check_id}", response_model=schema.Check)
def update_check(check_id: int, request: schema.CheckUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, check_id=check_id, request=request)


@router.post("/{check_id}/submit", response_model=schema.Check)
def submit_check(check_id: int, db: Session = Depends(get_db)):
    return controller.submit_check(db=db, check_id=check_id)


@router.post("/{check_id}/send", response_model=schema.Check)
def send_to_kitchen(check_id: int, db: Session = Depends(get_db)):
    """Submit if needed and promote pending items to preparing."""
    return controller.send_to_kitchen(db=db, check_id=check_id)


@router.delete("/{check_id}")
def delete_check(check_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, check_id=check_id)


# CheckItem endpoints within checks
@router.post("/{check_id}/items", response_model=order_item_schema.CheckItem)
def add_item_to_check(check_id: int, request: order_item_schema.CheckItemCreateDirect, db: Session = Depends(get_db)):
    """Add a menu item directly to a check"""
    return order_item_controller.add_item_to_check(db=db, check_id=check_id, request=request)



@router.post("/{check_id}/recalculate", response_model=schema.Check)
def recalculate_check_totals(check_id: int, db: Session = Depends(get_db)):
    """Recalculate check totals from order items"""
    return controller.update_check_totals(db=db, check_id=check_id)


@router.put("/{check_id}/close", response_model=schema.Check)
def close_check(check_id: int, db: Session = Depends(get_db)):
    """Close a paid check"""
    return controller.close_check(db=db, check_id=check_id)


@router.post("/{check_id}/payments", response_model=schema.Check)
async def create_payment_for_check(
    check_id: int,
    request: CheckPaymentCreate,
    db: Session = Depends(get_db),
):
    return await controller.create_payment_for_check(
        db=db,
        check_id=check_id,
        request=request,
    )


@router.put("/{check_id}/mark-paid", response_model=schema.Check)
def mark_check_paid(check_id: int, payment_id: int, db: Session = Depends(get_db)):
    """Mark check as paid - called by payment-service after successful payment"""
    return controller.mark_check_paid_by_payment_service(db=db, check_id=check_id, payment_id=payment_id)