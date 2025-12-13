from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from decimal import Decimal
from shared.dependencies.database import get_db
from ..schemas import checks as schema
from shared.schemas import orders as order_schema
from shared.schemas import order_items as order_item_schema
from ..schemas import payment_method as payment_schema
from ..controllers import checks as controller
from ..controllers import orders as order_controller
from ..controllers import order_items as order_item_controller

router = APIRouter(prefix="/checks", tags=["checks"])


@router.post("/", response_model=schema.Check)
def create_check(request: schema.CheckCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


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


@router.get("/session/{session_id}", response_model=list[schema.Check])
def get_checks_by_session(session_id: int, db: Session = Depends(get_db)):
    return controller.read_by_session(db, session_id=session_id)


@router.put("/{check_id}", response_model=schema.Check)
def update_check(check_id: int, request: schema.CheckUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, check_id=check_id, request=request)


@router.post("/{check_id}/submit", response_model=schema.Check)
def submit_check(check_id: int, db: Session = Depends(get_db)):
    return controller.submit_check(db=db, check_id=check_id)


@router.post("/{check_id}/payment", response_model=schema.Check)
def process_payment(check_id: int, tip_amount: Decimal = None, db: Session = Depends(get_db)):
    return controller.process_payment(db=db, check_id=check_id, tip_amount=tip_amount)


@router.delete("/{check_id}")
def delete_check(check_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, check_id=check_id)


# Order endpoints within checks
@router.post("/{check_id}/orders", response_model=order_schema.Order)
def create_order_for_check(check_id: int, request: order_schema.OrderCreate, db: Session = Depends(get_db)):
    # Ensure the order is created for this check
    request.check_id = check_id
    return order_controller.create(db=db, request=request)


@router.get("/{check_id}/orders", response_model=list[order_schema.Order])
def get_orders_for_check(check_id: int, db: Session = Depends(get_db)):
    return order_controller.read_by_check(db=db, check_id=check_id)


@router.get("/{check_id}/orders/{order_id}", response_model=order_schema.Order)
def get_order_in_check(check_id: int, order_id: int, db: Session = Depends(get_db)):
    return order_controller.read_one_in_check(db=db, check_id=check_id, order_id=order_id)


@router.post("/{check_id}/items", response_model=order_item_schema.OrderItem)
def add_item_to_check(check_id: int, request: order_item_schema.CheckItemCreate, db: Session = Depends(get_db)):
    """Add a menu item directly to a check"""
    return order_item_controller.add_item_to_check(db=db, check_id=check_id, request=request)


@router.put("/{check_id}/send", response_model=schema.Check)
def send_check_to_kitchen(check_id: int, db: Session = Depends(get_db)):
    """Send all unsent items in a check to kitchen"""
    return controller.send_check_to_kitchen(db=db, check_id=check_id)


@router.post("/{check_id}/recalculate", response_model=schema.Check)
def recalculate_check_totals(check_id: int, db: Session = Depends(get_db)):
    """Recalculate check totals from order items"""
    return controller.update_check_totals(db=db, check_id=check_id)


@router.post("/{check_id}/payments", response_model=schema.Check)
def create_payment_for_check(check_id: int, request: payment_schema.CheckPaymentCreate, db: Session = Depends(get_db)):
    """Create payment for a check"""
    return controller.create_payment_for_check(db=db, check_id=check_id, request=request)


@router.put("/{check_id}/close", response_model=schema.Check)
def close_check(check_id: int, db: Session = Depends(get_db)):
    """Close a paid check"""
    return controller.close_check(db=db, check_id=check_id)