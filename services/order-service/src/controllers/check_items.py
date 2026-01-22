from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response
from shared.models import check_items as model
from shared.models import menu_items as menu_model
from ..utils.errors import (
    handle_sqlalchemy_error,
    raise_not_found
)
from sqlalchemy.exc import SQLAlchemyError
from decimal import Decimal


def create(db: Session, request):
    """Create a check item directly for a check"""
    from ..models import checks as check_model
    
    check = db.query(check_model.Check).filter(check_model.Check.id == request.check_id).first()
    if not check:
        raise_not_found("Check", request.check_id)
    
    menu_item = db.query(menu_model.MenuItem).filter(menu_model.MenuItem.id == request.menu_item_id).first()
    if not menu_item:
        raise_not_found("Menu item", request.menu_item_id)
    
    # Calculate line total
    line_total = Decimal(str(menu_item.price)) * request.quantity

    new_item = model.CheckItem(
        check_id=request.check_id,
        menu_item_id=request.menu_item_id,
        quantity=request.quantity,
        unit_price=menu_item.price,
        total_price=line_total,
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
    """Create a check item for a check"""
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
    
    # Get the menu item to verify it exists and get price
    menu_item = db.query(menu_model.MenuItem).filter(menu_model.MenuItem.id == menu_item_id).first()
    if not menu_item:
        raise_not_found("Menu item", menu_item_id)
    
    # Calculate line total
    line_total = Decimal(str(menu_item.price)) * quantity

    # Create the check item
    new_item = model.CheckItem(
        check_id=check_id,
        menu_item_id=menu_item_id,
        quantity=quantity,
        unit_price=menu_item.price,
        total_price=line_total,
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
        query = db.query(model.CheckItem).options(
            joinedload(model.CheckItem.menu_item),
            joinedload(model.CheckItem.check)
        )
        if status:
            query = query.filter(model.CheckItem.status == status)
        if check_id:
            query = query.filter(model.CheckItem.check_id == check_id)
        result = query.all()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return result


def read_by_check(db: Session, check_id):
    """Get all check items for a specific check"""
    try:
        items = db.query(model.CheckItem).filter(model.CheckItem.check_id == check_id).all()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return items


def read_one(db: Session, item_id):
    try:
        item = db.query(model.CheckItem).filter(model.CheckItem.id == item_id).first()
        if not item:
            raise_not_found("Check item", item_id)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return item


def update(db: Session, item_id, request):
    try:
        item = db.query(model.CheckItem).filter(model.CheckItem.id == item_id)
        if not item.first():
            raise_not_found("Check item", item_id)
        
        current_item = item.first()
        update_data = request.model_dump(exclude_unset=True)
        
        # Update total if quantity changed
        if 'quantity' in update_data:
            update_data['total_price'] = current_item.unit_price * update_data['quantity']
        
        # Check if status is being changed to PREPARING and robot is enabled
        new_status = update_data.get('status')
        status_changed_to_preparing = (
            new_status is not None and 
            (new_status == model.CheckItemStatus.PREPARING or 
             (hasattr(new_status, 'value') and new_status.value == 'preparing')) and
            current_item.status != model.CheckItemStatus.PREPARING
        )
        
        item.update(update_data, synchronize_session=False)
        db.commit()
        
        # If item was just set to preparing and robot is enabled, add to robot queue
        # Note: Robot queue integration will be updated in a later task when robot_queue model is refactored
        if status_changed_to_preparing:
            from ..routers.robot import get_robot_enabled_state
            
            if get_robot_enabled_state():
                # TODO: Update robot queue integration after robot_queue model is refactored to use check_id
                pass
        
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return item.first()


def delete(db: Session, item_id):
    try:
        item = db.query(model.CheckItem).filter(model.CheckItem.id == item_id).first()
        if not item:
            raise_not_found("Check item", item_id)
        
        # Prevent deletion of preparing/ready items
        if item.status in [model.CheckItemStatus.PREPARING, model.CheckItemStatus.READY]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Cannot delete item with status '{item.status.value}'. Only pending items can be deleted."
            )
        
        db.delete(item)
        db.commit()
    except HTTPException:
        raise
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return Response(status_code=status.HTTP_204_NO_CONTENT)


def add_item_to_check(db: Session, check_id: int, request):
    """Add a menu item directly to a check"""
    from ..models import checks as check_model
    
    check = db.query(check_model.Check).filter(check_model.Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    # Validate check status
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
        line_total = Decimal(str(menu_item.price)) * request.quantity
        
        new_item = model.CheckItem(
            check_id=check_id,
            menu_item_id=request.menu_item_id,
            quantity=request.quantity,
            unit_price=menu_item.price,
            total_price=line_total,
            special_instructions=request.special_instructions,
            status=model.CheckItemStatus.PENDING
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
    
    all_items = db.query(model.CheckItem).filter(
        model.CheckItem.check_id == check_id
    ).all()
    
    if not all_items:
        return False
    
    # Update ready status
    all_ready = all(item.status == model.CheckItemStatus.READY for item in all_items)
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
    """Mark a preparing check item as ready"""
    from ..models.checks import Check
    
    try:
        item = db.query(model.CheckItem).filter(model.CheckItem.id == item_id).first()
        if not item:
            raise_not_found("Check item", item_id)
        
        # Validate that item is in 'preparing' status
        if item.status != model.CheckItemStatus.PREPARING:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Cannot mark item as ready. Item status is '{item.status.value}', must be 'preparing'."
            )
        
        item.status = model.CheckItemStatus.READY
        db.commit()
        db.refresh(item)
        
        # Check if all items in the check are now ready and update check status
        check_all_items_ready(db, item.check_id)
        
        return item
        
    except HTTPException:
        raise
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
