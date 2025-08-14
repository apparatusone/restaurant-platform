from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response
from ..models import orders as model
from ..models import checks as check_model
from sqlalchemy.exc import SQLAlchemyError
import random
import string


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


def update(db: Session, order_id, request):
    try:
        order = db.query(model.Order).filter(model.Order.id == order_id)
        if not order.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found")
        update_data = request.dict(exclude_unset=True)
        order.update(update_data, synchronize_session=False)
        db.commit()
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
