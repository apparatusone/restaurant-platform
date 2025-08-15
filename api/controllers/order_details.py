from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response
from ..models import order_details as model
from ..models import orders as order_model
from ..models import menu_items as menu_model
from sqlalchemy.exc import SQLAlchemyError
from decimal import Decimal


def create(db: Session, request):
    order = db.query(order_model.Order).filter(order_model.Order.id == request.order_id).first()
    if not order:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order not found")
    
    menu_item = db.query(menu_model.MenuItem).filter(menu_model.MenuItem.id == request.menu_item_id).first()
    if not menu_item:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Menu item not found")
    
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
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)

    return new_detail


def read_all(db: Session):
    try:
        result = db.query(model.OrderDetail).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return result


def read_by_order(db: Session, order_id):
    try:
        details = db.query(model.OrderDetail).filter(model.OrderDetail.order_id == order_id).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return details


def read_one(db: Session, detail_id):
    try:
        detail = db.query(model.OrderDetail).filter(model.OrderDetail.id == detail_id).first()
        if not detail:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order detail not found")
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return detail


def update(db: Session, detail_id, request):
    try:
        detail = db.query(model.OrderDetail).filter(model.OrderDetail.id == detail_id)
        if not detail.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order detail not found")
        
        update_data = request.dict(exclude_unset=True)
        
        # update total
        if 'quantity' in update_data:
            current_detail = detail.first()
            update_data['line_total'] = current_detail.unit_price * update_data['quantity']
        
        detail.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return detail.first()


def delete(db: Session, detail_id):
    try:
        detail = db.query(model.OrderDetail).filter(model.OrderDetail.id == detail_id)
        if not detail.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Order detail not found")
        detail.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return Response(status_code=status.HTTP_204_NO_CONTENT)
