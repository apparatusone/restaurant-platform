from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response
from ..models import order_items as model
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

    new_item = model.OrderItem(
        order_id=request.order_id,
        menu_item_id=request.menu_item_id,
        quantity=request.quantity,
        unit_price=menu_item.price,
        line_total=line_total,
        special_instructions=request.special_instructions
    )

    try:
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()

    return new_item


def read_all(db: Session):
    try:
        result = db.query(model.OrderItem).all()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return result


def read_by_order(db: Session, order_id):
    try:
        items = db.query(model.OrderItem).filter(model.OrderItem.order_id == order_id).all()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return items


def read_one(db: Session, item_id):
    try:
        item = db.query(model.OrderItem).filter(model.OrderItem.id == item_id).first()
        if not item:
            raise_not_found("Order item", item_id)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return item


def update(db: Session, item_id, request):
    try:
        item = db.query(model.OrderItem).filter(model.OrderItem.id == item_id)
        if not item.first():
            raise_not_found("Order item", item_id)
        
        update_data = request.dict(exclude_unset=True)
        
        # update total
        if 'quantity' in update_data:
            current_item = item.first()
            update_data['line_total'] = current_item.unit_price * update_data['quantity']
        
        item.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return item.first()


def delete(db: Session, item_id):
    try:
        item = db.query(model.OrderItem).filter(model.OrderItem.id == item_id).first()
        if not item:
            raise_not_found("Order item", item_id)
        
        # prevent deletion of sent/ready
        if item.status in [model.OrderItemStatus.SENT, model.OrderItemStatus.READY]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Cannot delete item with status '{item.status.value}'. Only unsent items can be deleted."
            )
        
        db.delete(item)
        db.commit()
    except HTTPException:
        raise
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return Response(status_code=status.HTTP_204_NO_CONTENT)


def add_item_to_check(db: Session, check_id: int, request):
    """Add a menu item directly to a check by creating an order if needed"""
    from ..models import checks as check_model
    from ..models import orders as order_model
    from ..models import menu_items as menu_model
    
    check = db.query(check_model.Check).filter(check_model.Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    # validate check status
    valid_statuses = [check_model.CheckStatus.OPEN, check_model.CheckStatus.SENT, check_model.CheckStatus.READY]
    if check.status not in valid_statuses:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot add items to check with status '{check.status.value}'. Check must be open, sent, or ready."
        )
    
    menu_item = db.query(menu_model.MenuItem).filter(menu_model.MenuItem.id == request.menu_item_id).first()
    if not menu_item:
        raise_not_found("Menu item", request.menu_item_id)
    
    try:
        # find or create an order for this check
        order = db.query(order_model.Order).filter(order_model.Order.check_id == check_id).first()
        
        if not order:
            # create a new order
            order = order_model.Order(
                check_id=check_id,
                status=order_model.OrderStatus.PENDING,
                order_type=order_model.OrderType.DINE_IN  # Default for table orders
            )
            db.add(order)
            db.commit()
            db.refresh(order)
        
        line_total = Decimal(str(menu_item.price)) * request.quantity
        
        new_item = model.OrderItem(
            order_id=order.id,
            menu_item_id=request.menu_item_id,
            quantity=request.quantity,
            unit_price=menu_item.price,
            line_total=line_total,
            special_instructions=request.special_instructions,
            status=model.OrderItemStatus.UNSENT
        )
        
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
        
        return new_item
        
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()