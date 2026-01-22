from sqlalchemy.orm import Session, joinedload
from sqlalchemy.exc import SQLAlchemyError
from shared.models.check_items import CheckItem, CheckItemStatus
from shared.models.menu_items import MenuItem

# Import Check model to ensure relationship is available
# Kitchen service needs this for joinedload operations
try:
    from services.order_service.src.models.checks import Check
except ImportError:
    # If we can't import from order service, define a minimal stub
    # This allows CheckItem relationships to work
    from sqlalchemy import Column, Integer, String, DateTime, Numeric, Enum, ForeignKey, Boolean
    from sqlalchemy.orm import relationship
    from sqlalchemy.sql import func
    from shared.dependencies.database import Base
    import enum
    
    class CheckStatus(enum.Enum):
        OPEN = "open"
        SENT = "sent"
        READY = "ready"
        PAID = "paid"
        CLOSED = "closed"
    
    class Check(Base):
        __tablename__ = "checks"
        id = Column(Integer, primary_key=True)
        session_id = Column(Integer, nullable=True)
        is_virtual = Column(Boolean, default=False)
        status = Column(Enum(CheckStatus, values_callable=lambda obj: [e.value for e in obj]), default=CheckStatus.OPEN)
        subtotal = Column(Numeric(10, 2), default=0.00)
        tax_amount = Column(Numeric(10, 2), default=0.00)
        tip_amount = Column(Numeric(10, 2), default=0.00)
        total_amount = Column(Numeric(10, 2), default=0.00)
        created_at = Column(DateTime(timezone=True), server_default=func.now())
        updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
        check_items = relationship("CheckItem", back_populates="check")


def get_kitchen_queue(db: Session):
    """Get all checks currently in kitchen (pending, confirmed, in_progress)"""
    try:
        # This function is deprecated - kitchen now works with check items directly
        # Use get_kitchen_items() instead
        raise NotImplementedError("Use get_kitchen_items() to get items in kitchen")
    except SQLAlchemyError as e:
        raise Exception(f"Database error: {str(e)}")


def get_kitchen_items(db: Session):
    """Get all check items that are PREPARING (in kitchen prep)"""
    try:
        items = db.query(CheckItem).options(
            joinedload(CheckItem.menu_item),
            joinedload(CheckItem.check)
        ).filter(
            CheckItem.status == CheckItemStatus.PREPARING
        ).all()
        return items
    except SQLAlchemyError as e:
        raise Exception(f"Database error: {str(e)}")


def mark_item_ready(db: Session, item_id: int):
    """Mark a check item as ready"""
    item = db.query(CheckItem).filter(CheckItem.id == item_id).first()
    if not item:
        raise ValueError(f"Check item {item_id} not found")
    
    if item.status != CheckItemStatus.PREPARING:
        raise ValueError(f"Can only mark PREPARING items as ready. Current status: {item.status.value}")
    
    item.status = CheckItemStatus.READY
    db.commit()
    db.refresh(item)
    
    # TODO: Publish kitchen.item_ready event to message bus
    
    return item


def mark_items_ready_bulk(db: Session, item_ids: list[int]):
    """Mark multiple check items as ready"""
    items = db.query(CheckItem).filter(CheckItem.id.in_(item_ids)).all()
    
    if len(items) != len(item_ids):
        raise ValueError("Some items not found")
    
    for item in items:
        if item.status != CheckItemStatus.PREPARING:
            raise ValueError(f"Item {item.id} is not PREPARING (current: {item.status.value})")
        item.status = CheckItemStatus.READY
    
    db.commit()
    
    # TODO: Publish kitchen.item_ready events
    
    return items
