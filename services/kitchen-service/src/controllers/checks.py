from sqlalchemy.orm import Session
from datetime import datetime
from shared.models.check_items import CheckItem, CheckItemStatus


def send_check_to_kitchen(db: Session, check_id: int):
    """Send all pending items in a check to kitchen"""
    # Import Check model from order-service's models
    # Note: This is a shared DB access pattern
    from services.order_service.src.models.checks import Check, CheckStatus
    
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise ValueError(f"Check {check_id} not found")
    
    pending_items = db.query(CheckItem).filter(
        CheckItem.check_id == check_id,
        CheckItem.status == CheckItemStatus.PENDING
    ).all()
    
    if not pending_items:
        raise ValueError("No pending items found in this check")
    
    # Mark all pending items as preparing (now in kitchen)
    for item in pending_items:
        item.status = CheckItemStatus.PREPARING
    
    # If check is open, mark status as sent
    if check.status == CheckStatus.OPEN:
        check.status = CheckStatus.SENT
        check.submitted_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    
    # TODO: Publish kitchen.items_sent event to message bus
    
    return check
