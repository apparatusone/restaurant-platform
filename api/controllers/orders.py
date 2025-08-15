from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response
from ..models import orders as model
from ..models import checks as check_model
from sqlalchemy.exc import SQLAlchemyError
import random
import string
from datetime import datetime
from typing import List


# order status transition validation
ORDER_STATUS_TRANSITIONS = {
    model.OrderStatus.PENDING: [model.OrderStatus.CONFIRMED, model.OrderStatus.CANCELLED],
    model.OrderStatus.CONFIRMED: [model.OrderStatus.IN_PROGRESS, model.OrderStatus.CANCELLED],
    model.OrderStatus.IN_PROGRESS: [model.OrderStatus.READY],
    model.OrderStatus.READY: [model.OrderStatus.SERVED, model.OrderStatus.AWAITING_PICKUP, model.OrderStatus.OUT_FOR_DELIVERY],
    model.OrderStatus.AWAITING_PICKUP: [model.OrderStatus.COMPLETED],
    model.OrderStatus.OUT_FOR_DELIVERY: [model.OrderStatus.DELIVERED],
    model.OrderStatus.SERVED: [model.OrderStatus.COMPLETED],
    model.OrderStatus.DELIVERED: [],
    model.OrderStatus.COMPLETED: [],
    model.OrderStatus.CANCELLED: []
}


def validate_status_transition(current_status: model.OrderStatus, new_status: model.OrderStatus) -> bool:
    if current_status not in ORDER_STATUS_TRANSITIONS:
        return False
    return new_status in ORDER_STATUS_TRANSITIONS[current_status]


def get_valid_next_statuses(current_status: model.OrderStatus) -> List[model.OrderStatus]:
    return ORDER_STATUS_TRANSITIONS.get(current_status, [])


def generate_tracking_number(order_type: model.OrderType) -> str:
    """Generate a tracking number for takeout and delivery orders"""
    if order_type == model.OrderType.TAKEOUT:
        prefix = "TK"
    elif order_type == model.OrderType.DELIVERY:
        prefix = "DL"
    else:
        return None
    
    # generate random alphanumeric string
    suffix = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
    return f"{prefix}{suffix}"


def create(db: Session, request):
    check = db.query(check_model.Check).filter(check_model.Check.id == request.check_id).first()
    if not check:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Check not found")
    
    tracking_number = request.tracking_number
    if not tracking_number and request.order_type in [model.OrderType.TAKEOUT, model.OrderType.DELIVERY]:
        tracking_number = generate_tracking_number(request.order_type)
    
    new_order = model.Order(
        check_id=request.check_id,
        customer_id=request.customer_id,
        promo_id=request.promo_id,
        status=request.status,
        order_type=request.order_type,
        tracking_number=tracking_number,
        notes=request.notes
    )

    try:
        db.add(new_order)
        db.commit()
        db.refresh(new_order)
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)

    return new_order


def read_all(db: Session):
    try:
        result = db.query(model.Order).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return result


def read_one(db: Session, order_id):
    try:
        order = db.query(model.Order).filter(model.Order.id == order_id).first()
        if not order:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found")
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return order


def get_order_status_info(db: Session, order_id: int):
    try:
        order = db.query(model.Order).filter(model.Order.id == order_id).first()
        if not order:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found")
        
        valid_next_statuses = get_valid_next_statuses(order.status)
        
        return {
            "order": order,
            "current_status": order.status.value,
            "valid_next_states": [s.value for s in valid_next_statuses]
        }
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)


def read_by_check(db: Session, check_id):
    try:
        orders = db.query(model.Order).filter(model.Order.check_id == check_id).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return orders


def read_one_in_check(db: Session, check_id, order_id):
    try:
        order = db.query(model.Order).filter(
            model.Order.id == order_id,
            model.Order.check_id == check_id
        ).first()
        if not order:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found in this check")
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return order


def get_kitchen_orders(db: Session):
    try:
        orders = db.query(model.Order).filter(
            model.Order.status.in_([
                model.OrderStatus.PENDING,
                model.OrderStatus.CONFIRMED,
                model.OrderStatus.IN_PROGRESS
            ])
        ).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return orders


def get_delivery_orders(db: Session):
    try:
        orders = db.query(model.Order).filter(
            model.Order.status.in_([
                model.OrderStatus.OUT_FOR_DELIVERY
            ])
        ).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return orders


def get_pickup_orders(db: Session):
    try:
        orders = db.query(model.Order).filter(
            model.Order.status.in_([
                model.OrderStatus.AWAITING_PICKUP
            ])
        ).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return orders


def update_status(db: Session, order_id: int, new_status: model.OrderStatus):
    try:
        order = db.query(model.Order).filter(model.Order.id == order_id).first()
        if not order:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found")
        
        # validate status
        if not validate_status_transition(order.status, new_status):
            valid_statuses = get_valid_next_statuses(order.status)
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": "Invalid status transition",
                    "current_status": order.status.value,
                    "requested_status": new_status.value,
                    "valid_next_states": [s.value for s in valid_statuses]
                }
            )
        
        order.status = new_status
        order.updated_at = datetime.utcnow()
        db.commit()
        db.refresh(order)
        
        return order
        
    except HTTPException:
        raise
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)


def update(db: Session, order_id, request):
    try:
        order = db.query(model.Order).filter(model.Order.id == order_id)
        if not order.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found")
        
        update_data = request.dict(exclude_unset=True)
        
        # if status is being updated, validate
        if 'status' in update_data:
            current_order = order.first()
            new_status = update_data['status']
            if not validate_status_transition(current_order.status, new_status):
                valid_statuses = get_valid_next_statuses(current_order.status)
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail={
                        "error": "Invalid status transition",
                        "current_status": current_order.status.value,
                        "requested_status": new_status.value,
                        "valid_next_states": [s.value for s in valid_statuses]
                    }
                )
        
        order.update(update_data, synchronize_session=False)
        db.commit()
    except HTTPException:
        raise
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return order.first()


def delete(db: Session, order_id):
    try:
        order = db.query(model.Order).filter(model.Order.id == order_id)
        if not order.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found")
        order.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return Response(status_code=status.HTTP_204_NO_CONTENT)
