from sqlalchemy.orm import Session, joinedload
from sqlalchemy.exc import SQLAlchemyError
from shared.models.orders import Order, OrderStatus
from shared.models.order_items import OrderItem, OrderItemStatus
from shared.models.menu_items import MenuItem


def get_kitchen_queue(db: Session):
    """Get all orders currently in kitchen (pending, confirmed, in_progress)"""
    try:
        orders = db.query(Order).options(
            joinedload(Order.order_items).joinedload(OrderItem.menu_item)
        ).filter(
            Order.status.in_([
                OrderStatus.PENDING,
                OrderStatus.CONFIRMED,
                OrderStatus.IN_PROGRESS
            ])
        ).all()
        return orders
    except SQLAlchemyError as e:
        raise Exception(f"Database error: {str(e)}")


def get_kitchen_items(db: Session):
    """Get all order items that are SENT (in kitchen prep)"""
    try:
        items = db.query(OrderItem).options(
            joinedload(OrderItem.menu_item),
            joinedload(OrderItem.order)
        ).filter(
            OrderItem.status == OrderItemStatus.SENT
        ).all()
        return items
    except SQLAlchemyError as e:
        raise Exception(f"Database error: {str(e)}")


def mark_item_ready(db: Session, item_id: int):
    """Mark an order item as ready"""
    item = db.query(OrderItem).filter(OrderItem.id == item_id).first()
    if not item:
        raise ValueError(f"Order item {item_id} not found")
    
    if item.status != OrderItemStatus.SENT:
        raise ValueError(f"Can only mark SENT items as ready. Current status: {item.status.value}")
    
    item.status = OrderItemStatus.READY
    db.commit()
    db.refresh(item)
    
    # TODO: Publish kitchen.item_ready event to message bus
    
    return item


def mark_items_ready_bulk(db: Session, item_ids: list[int]):
    """Mark multiple order items as ready"""
    items = db.query(OrderItem).filter(OrderItem.id.in_(item_ids)).all()
    
    if len(items) != len(item_ids):
        raise ValueError("Some items not found")
    
    for item in items:
        if item.status != OrderItemStatus.SENT:
            raise ValueError(f"Item {item.id} is not SENT (current: {item.status.value})")
        item.status = OrderItemStatus.READY
    
    db.commit()
    
    # TODO: Publish kitchen.item_ready events
    
    return items
