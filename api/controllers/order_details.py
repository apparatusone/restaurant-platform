from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response
from ..models import order_details as model
from ..models import orders as order_model
from ..models import menu_items as menu_model
from ..utils.errors import (
    handle_sqlalchemy_error,
    raise_not_found
)
from sqlalchemy.exc import SQLAlchemyError
from decimal import Decimal


def create(db: Session, request):
    order = db.query(order_model.Order).filter(order_model.Order.id == request.order_id).first()
    if not order:
        raise_not_found("Order", request.order_id)
    
    menu_item = db.query(menu_model.MenuItem).filter(menu_model.MenuItem.id == request.menu_item_id).first()
    if not menu_item:
        raise_not_found("Menu item", request.menu_item_id)
    
    # initial total
    line_total = Decimal(str(menu_item.price)) * request.quantity

    new_detail = model.OrderDetail(
        order_id=request.order_id,
        menu_item_id=request.menu_item_id,
        quantity=request.quantity,
        unit_price=menu_item.price,
        line_total=line_total,
        special_instructions=request.special_instructions
    )

    try:
        db.add(new_detail)
        db.commit()
        db.refresh(new_detail)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()

    return new_detail


def read_all(db: Session):
    try:
        result = db.query(model.OrderDetail).all()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return result


def read_by_order(db: Session, order_id):
    try:
        details = db.query(model.OrderDetail).filter(model.OrderDetail.order_id == order_id).all()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return details


def read_one(db: Session, detail_id):
    try:
        detail = db.query(model.OrderDetail).filter(model.OrderDetail.id == detail_id).first()
        if not detail:
            raise_not_found("Order detail", detail_id)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return detail


def update(db: Session, detail_id, request):
    try:
        detail = db.query(model.OrderDetail).filter(model.OrderDetail.id == detail_id)
        if not detail.first():
            raise_not_found("Order detail", detail_id)
        
        update_data = request.dict(exclude_unset=True)
        
        # update total
        if 'quantity' in update_data:
            current_detail = detail.first()
            update_data['line_total'] = current_detail.unit_price * update_data['quantity']
        
        detail.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return detail.first()


def delete(db: Session, detail_id):
    try:
        detail = db.query(model.OrderDetail).filter(model.OrderDetail.id == detail_id)
        if not detail.first():
            raise_not_found("Order detail", detail_id)
        detail.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return Response(status_code=status.HTTP_204_NO_CONTENT)
