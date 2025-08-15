from fastapi import APIRouter, Depends, Query, Path
from sqlalchemy.orm import Session
from datetime import date
from ..controllers import orders as controller
from ..schemas import orders as schema
from ..models.orders import OrderStatus
from ..services import staff as staff_services
from ..services import customer as customer_services

from ..dependencies.database import get_db

router = APIRouter(prefix="/orders", tags=["orders"])


@router.post("/", response_model=schema.Order)
def create_order(request: schema.OrderCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


# REST endpoints for order filtering and querying
@router.get("/", response_model=list[schema.Order])
def get_orders(
    status: OrderStatus = Query(None, description="Filter orders by status"),
    start_date: date = Query(None, description="Start date for date range filter (YYYY-MM-DD)"),
    end_date: date = Query(None, description="End date for date range filter (YYYY-MM-DD)"),
    db: Session = Depends(get_db)
):
    """
    Get orders with optional filtering by status or date range
    """
    if status:
        return staff_services.get_orders_by_status(db=db, status=status)
    elif start_date and end_date:
        return staff_services.get_orders_by_date_range(db=db, start_date=start_date, end_date=end_date)
    else:
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


# Order tracking endpoint
@router.get("/track/{tracking_number}")
def track_order(tracking_number: str, db: Session = Depends(get_db)):
    """
    Get order tracking information using tracking number
    """
    return customer_services.get_tracking_information(
        db=db,
        tracking_number=tracking_number
    )


@router.get("/{order_id}", response_model=schema.Order)
def get_order(order_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, order_id=order_id)


@router.put("/{order_id}", response_model=schema.Order)
def update_order(order_id: int, request: schema.OrderUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, order_id=order_id, request=request)


@router.delete("/{order_id}")
def delete_order(order_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, order_id=order_id)


# REST endpoints for order status management
@router.get("/{order_id}/status")
def get_order_status_info(order_id: int, db: Session = Depends(get_db)):
    """
    Get order status information with valid next states
    """
    return controller.get_order_status_info(db=db, order_id=order_id)


@router.put("/{order_id}/status")
def update_order_status(order_id: int, status: OrderStatus, db: Session = Depends(get_db)):
    """
    Update the status of an order with validation
    """
    return staff_services.update_order_status(db=db, order_id=order_id, status=status)

