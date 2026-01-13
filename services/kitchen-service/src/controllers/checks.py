from sqlalchemy.orm import Session
from datetime import datetime
from shared.models.order_items import OrderItem, OrderItemStatus
from shared.models.orders import Order


def send_check_to_kitchen(db: Session, check_id: int):
    """Send all unsent items in a check to kitchen"""
    # Import Check model from order-service's models
    # Note: This is a shared DB access pattern
    from services.order_service.src.models.checks import Check, CheckStatus
    
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise ValueError(f"Check {check_id} not found")
    
    unsent_items = db.query(OrderItem).join(Order).filter(
        Order.check_id == check_id,
        OrderItem.status == OrderItemStatus.UNSENT
    ).all()
    
    if not unsent_items:
        raise ValueError("No unsent items found in this check")
    
    # Mark all unsent items as sent (now in kitchen)
    for item in unsent_items:
        item.status = OrderItemStatus.SENT
    
    # If check is open, mark status as sent
    if check.status == CheckStatus.OPEN:
        check.status = CheckStatus.SENT
        check.submitted_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    
    # TODO: Publish kitchen.items_sent event to message bus
    
    return check
