from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response
from shared.models import order_items as model
from shared.models import orders as order_model
from shared.models import menu_items as menu_model
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


def create_for_check(db: Session, check_id: int, menu_item_id: int, quantity: int, special_instructions: str = None):
    """Create an order item for a check's order (handles 1-1 check-order relationship)"""
    from ..models import checks as check_model
    from ..models.checks import CheckStatus
    
    # Get the check
    check = db.query(check_model.Check).filter(check_model.Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    # Prevent adding items to paid or closed checks
    if check.status in [CheckStatus.PAID, CheckStatus.CLOSED]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Cannot add items to a check with status '{check.status.value}'. Check is already {check.status.value}."
        )
    
    # Get the check's order (with 1-1 relationship, this should be a single order)
    order = check.order
    if not order:
        # If no order exists for this check, we need to create one
        # This might happen for checks created without orders initially
        from shared.models.orders import Order, OrderStatus, OrderType
        
        # Create a default order for the check
        order = Order(
            check_id=check_id,
            status=OrderStatus.PENDING,
            order_type=OrderType.DINE_IN  # default, could be made configurable
        )
        
        try:
            db.add(order)
            db.commit()
            db.refresh(order)
        except SQLAlchemyError as e:
            handle_sqlalchemy_error(e).raise_exception()
    
    # Get the menu item to verify it exists and get price
    menu_item = db.query(menu_model.MenuItem).filter(menu_model.MenuItem.id == menu_item_id).first()
    if not menu_item:
        raise_not_found("Menu item", menu_item_id)
    
    # Calculate line total
    line_total = Decimal(str(menu_item.price)) * quantity

    # Create the order item
    new_item = model.OrderItem(
        order_id=order.id,
        menu_item_id=menu_item_id,
        quantity=quantity,
        unit_price=menu_item.price,
        line_total=line_total,
        special_instructions=special_instructions
    )

    try:
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()

    return new_item


def read_all(db: Session, status: str = None, check_id: int = None):
    from sqlalchemy.orm import joinedload
    try:
        query = db.query(model.OrderItem).options(
            joinedload(model.OrderItem.menu_item),
            joinedload(model.OrderItem.order).joinedload(order_model.Order.check)
        )
        if status:
            query = query.filter(model.OrderItem.status == status)
        if check_id:
            query = query.join(order_model.Order).filter(order_model.Order.check_id == check_id)
        result = query.all()
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
        
        current_item = item.first()
        update_data = request.dict(exclude_unset=True)
        
        # update total
        if 'quantity' in update_data:
            update_data['line_total'] = current_item.unit_price * update_data['quantity']
        
        # Check if status is being changed to SENT and robot is enabled
        # Compare enum values (strings) instead of enum objects
        new_status = update_data.get('status')
        status_changed_to_sent = (
            new_status is not None and 
            (new_status == model.OrderItemStatus.SENT or 
             (hasattr(new_status, 'value') and new_status.value == 'sent')) and
            current_item.status != model.OrderItemStatus.SENT
        )
        
        item.update(update_data, synchronize_session=False)
        db.commit()
        
        # If item was just sent and robot is enabled, add to robot queue
        if status_changed_to_sent:
            from ..routers.robot import get_robot_enabled_state
            
            if get_robot_enabled_state():
                from ..services.robot import robot_service
                from ..models.robot_queue import RobotQueue
                
                order_id = current_item.order_id
                
                # Check if order is already in queue
                existing_queue_entry = db.query(RobotQueue).filter(
                    RobotQueue.order_id == order_id
                ).first()
                
                # Only add if not already in queue
                if not existing_queue_entry:
                    robot_service.add_to_queue(db, order_id)
        
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


def check_all_items_ready(db: Session, check_id: int):
    """Check if all items in a check are ready and update check status accordingly"""
    from ..models.checks import Check, CheckStatus
    from shared.models.orders import Order
    
    all_items = db.query(model.OrderItem).join(Order).filter(
        Order.check_id == check_id
    ).all()
    
    if not all_items:
        return False
    
    # update ready status
    all_ready = all(item.status == model.OrderItemStatus.READY for item in all_items)
    if all_ready:
        check = db.query(Check).filter(Check.id == check_id).first()
        # Only update status to READY if check is still in SENT status
        # Don't change status if already PAID or CLOSED
        if check and check.status == CheckStatus.SENT:
            check.status = CheckStatus.READY
            db.commit()
            db.refresh(check)
    
    return all_ready


def mark_item_ready(db: Session, item_id: int):
    """Mark a sent order item as ready"""
    from ..models.checks import Check
    from shared.models.orders import Order
    
    try:
        item = db.query(model.OrderItem).filter(model.OrderItem.id == item_id).first()
        if not item:
            raise_not_found("Order item", item_id)
        
        # validate that item is in 'sent' status
        if item.status != model.OrderItemStatus.SENT:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Cannot mark item as ready. Item status is '{item.status.value}', must be 'sent'."
            )
        
        item.status = model.OrderItemStatus.READY
        db.commit()
        db.refresh(item)
        
        # check if all items in the check are now ready and update check status
        order = db.query(Order).filter(Order.id == item.order_id).first()
        if order:
            check_all_items_ready(db, order.check_id)
        
        return item
        
    except HTTPException:
        raise
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()