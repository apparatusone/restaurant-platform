from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import json
import os

from ..dependencies.database import get_db
from ..services.robot import robot_service
from ..models.orders import Order
from ..models.order_items import OrderItem
from ..models.menu_items import MenuItem
from ..models.robot_queue import RobotQueue

router = APIRouter(prefix="/robot", tags=["robot"])

# Path to config file
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "restaurant_config.json")

def get_robot_enabled_state() -> bool:
    """Get the robot_kitchen_enabled state from config"""
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return config.get("business_settings", {}).get("robot_kitchen_enabled", False)
    except Exception as e:
        print(f"Error reading config: {e}")
        return False

def set_robot_enabled_state(enabled: bool) -> bool:
    """Set the robot_kitchen_enabled state in config"""
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = json.load(f)
        
        if "business_settings" not in config:
            config["business_settings"] = {}
        
        config["business_settings"]["robot_kitchen_enabled"] = enabled
        
        with open(CONFIG_PATH, 'w') as f:
            json.dump(config, f, indent=4)
        
        return True
    except Exception as e:
        print(f"Error writing config: {e}")
        return False

@router.get("/enabled")
def get_robot_enabled():
    """Get whether robot kitchen integration is enabled"""
    return {
        "robot_kitchen_enabled": get_robot_enabled_state()
    }

@router.post("/enabled")
def set_robot_enabled(enabled: bool):
    """Enable or disable robot kitchen integration"""
    success = set_robot_enabled_state(enabled)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update configuration")
    
    return {
        "robot_kitchen_enabled": enabled,
        "message": f"Robot kitchen {'enabled' if enabled else 'disabled'}"
    }


@router.post("/send-order/{order_id}")
async def send_order_to_robot(
    order_id: int,
    db: Session = Depends(get_db),
    force: bool = False  # Force send even if robot is busy
):
    """Send an existing order to the robot kitchen"""
    from ..models.robot_queue import RobotQueue, RobotQueueStatus
    
    # Check if robot is enabled
    if not get_robot_enabled_state():
        raise HTTPException(
            status_code=400, 
            detail="Robot kitchen integration is disabled. Enable it in the Robot section to send orders."
        )
    
    # Fetch the order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")
    
    # Add to queue if not already there, or reset to pending if it was confirmed
    queue_entry = robot_service.add_to_queue(db, order_id)
    if queue_entry.status == RobotQueueStatus.CONFIRMED:
        # Reset confirmed orders back to pending for re-sending
        queue_entry.status = RobotQueueStatus.PENDING
        queue_entry.error_message = None
        db.commit()
    
    # Check if robot is busy (unless force is True)
    if not force:
        is_busy = await robot_service.is_robot_busy()
        if is_busy:
            return {
                "order_id": order_id,
                "message": "Robot is busy, order queued for later processing",
                "robot_busy": True,
                "queue_status": robot_service.get_queue_status(db, order_id)
            }
    
    # Fetch order items with menu item details - only sent items that haven't been sent to robot yet
    from ..models.order_items import OrderItemStatus
    order_items = db.query(OrderItem).filter(
        OrderItem.order_id == order_id,
        OrderItem.status.in_([OrderItemStatus.SENT, OrderItemStatus.UNSENT])  # Include unsent for manual sends
    ).all()
    
    # Filter out items that have already been sent to robot (status is READY means robot processed it)
    order_items = [item for item in order_items if item.status != OrderItemStatus.READY]
    
    # Prepare items data with resources
    from ..models.menu_item_ingredients import MenuItemIngredient
    from ..models.resources import Resource
    
    items = []
    for item in order_items:
        menu_item = db.query(MenuItem).filter(MenuItem.id == item.menu_item_id).first()
        
        # Get resources for this menu item
        resources = []
        if menu_item:
            ingredients = db.query(MenuItemIngredient).filter(
                MenuItemIngredient.menu_item_id == menu_item.id
            ).all()
            
            for ingredient in ingredients:
                resource = db.query(Resource).filter(Resource.id == ingredient.resource_id).first()
                if resource:
                    resources.append({
                        "resource_id": resource.id,
                        "name": resource.name,
                        "amount": ingredient.amount
                    })
        
        items.append({
            "item_id": item.menu_item_id,
            "name": menu_item.name if menu_item else "Unknown",
            "quantity": item.quantity,
            "special_instructions": item.special_instructions or "",
            "resources": resources
        })
    
    # Send to robot (with db session for queue tracking)
    result = await robot_service.send_order_to_robot(order_id, items, db)
    
    # If successful, update order items status to READY (robot has them)
    if result.get("status") == "success":
        for item in order_items:
            item.status = OrderItemStatus.READY
        db.commit()
    
    # Get queue status
    queue_status = robot_service.get_queue_status(db, order_id)
    
    return {
        "order_id": order_id,
        "robot_response": result,
        "robot_busy": False,
        "queue_status": queue_status
    }

@router.get("/health")
async def check_robot_health():
    """Check if robot kitchen is available"""
    is_healthy = await robot_service.check_robot_health()
    return {
        "robot_kitchen_available": is_healthy,
        "status": "healthy" if is_healthy else "unavailable"
    }

@router.get("/status")
async def get_robot_status():
    """Get the current status of the robot kitchen"""
    status = await robot_service.get_robot_status()
    is_busy = await robot_service.is_robot_busy()
    return {
        **status,
        "is_busy": is_busy
    }

@router.get("/queue")
def get_queue(
    db: Session = Depends(get_db),
    limit: int = 50,
    include_confirmed: bool = False
):
    """Get orders in the robot queue (excludes confirmed by default)"""
    from ..models.robot_queue import RobotQueue, RobotQueueStatus
    
    query = db.query(RobotQueue)
    
    # By default, don't show confirmed orders (they're done)
    if not include_confirmed:
        query = query.filter(RobotQueue.status != RobotQueueStatus.CONFIRMED)
    
    queue_entries = query.order_by(RobotQueue.created_at.desc()).limit(limit).all()
    
    return [
        {
            "order_id": entry.order_id,
            "status": entry.status.value,
            "retry_count": entry.retry_count,
            "last_attempt_at": entry.last_attempt_at.isoformat() if entry.last_attempt_at else None,
            "sent_at": entry.sent_at.isoformat() if entry.sent_at else None,
            "confirmed_at": entry.confirmed_at.isoformat() if entry.confirmed_at else None,
            "error_message": entry.error_message,
            "created_at": entry.created_at.isoformat()
        }
        for entry in queue_entries
    ]

@router.get("/queue/pending")
def get_pending_queue(
    db: Session = Depends(get_db),
    limit: int = 10
):
    """Get pending orders in the robot queue"""
    pending = robot_service.get_pending_orders(db, limit)
    
    return [
        {
            "order_id": entry.order_id,
            "status": entry.status.value,
            "retry_count": entry.retry_count,
            "created_at": entry.created_at.isoformat()
        }
        for entry in pending
    ]

@router.get("/queue/{order_id}")
def get_queue_status(
    order_id: int,
    db: Session = Depends(get_db)
):
    """Get the queue status for a specific order"""
    status = robot_service.get_queue_status(db, order_id)
    if not status:
        raise HTTPException(status_code=404, detail="Order not found in queue")
    return status

@router.post("/queue/retry/{order_id}")
async def retry_order(
    order_id: int,
    db: Session = Depends(get_db)
):
    """Retry sending a failed order"""
    from ..models.robot_queue import RobotQueue, RobotQueueStatus
    
    queue_entry = db.query(RobotQueue).filter(RobotQueue.order_id == order_id).first()
    if not queue_entry:
        raise HTTPException(status_code=404, detail="Order not found in queue")
    
    if queue_entry.retry_count >= robot_service.max_retries:
        raise HTTPException(
            status_code=400, 
            detail=f"Max retries ({robot_service.max_retries}) exceeded"
        )
    
    # Reset to pending
    queue_entry.status = RobotQueueStatus.PENDING
    queue_entry.error_message = None
    db.commit()
    
    # Try to send again
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")
    
    order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    items = []
    for item in order_items:
        menu_item = db.query(MenuItem).filter(MenuItem.id == item.menu_item_id).first()
        items.append({
            "item_id": item.menu_item_id,
            "name": menu_item.name if menu_item else "Unknown",
            "quantity": item.quantity,
            "special_instructions": item.special_instructions or ""
        })
    
    result = await robot_service.send_order_to_robot(order_id, items, db)
    queue_status = robot_service.get_queue_status(db, order_id)
    
    return {
        "order_id": order_id,
        "robot_response": result,
        "queue_status": queue_status
    }

@router.post("/queue/process-pending")
async def process_pending_orders(
    db: Session = Depends(get_db),
    max_orders: int = 5
):
    """Process pending orders in the queue (send them to robot if not busy)"""
    
    # Check if robot is available
    is_busy = await robot_service.is_robot_busy()
    if is_busy:
        return {
            "message": "Robot is currently busy",
            "processed": 0,
            "robot_busy": True
        }
    
    # Get pending orders
    pending = robot_service.get_pending_orders(db, limit=max_orders)
    
    if not pending:
        return {
            "message": "No pending orders in queue",
            "processed": 0,
            "robot_busy": False
        }
    
    processed = []
    failed = []
    
    for queue_entry in pending:
        order = db.query(Order).filter(Order.id == queue_entry.order_id).first()
        if not order:
            failed.append({"order_id": queue_entry.order_id, "reason": "Order not found"})
            continue
        
        # Get order items
        order_items = db.query(OrderItem).filter(OrderItem.order_id == queue_entry.order_id).all()
        items = []
        for item in order_items:
            menu_item = db.query(MenuItem).filter(MenuItem.id == item.menu_item_id).first()
            items.append({
                "item_id": item.menu_item_id,
                "name": menu_item.name if menu_item else "Unknown",
                "quantity": item.quantity,
                "special_instructions": item.special_instructions or ""
            })
        
        # Try to send
        result = await robot_service.send_order_to_robot(queue_entry.order_id, items, db)
        
        if result.get("status") == "success":
            processed.append(queue_entry.order_id)
        else:
            failed.append({"order_id": queue_entry.order_id, "reason": result.get("message", "Unknown error")})
        
        # If robot becomes busy, stop processing
        if await robot_service.is_robot_busy():
            break
    
    return {
        "message": f"Processed {len(processed)} orders",
        "processed": processed,
        "failed": failed,
        "robot_busy": await robot_service.is_robot_busy()
    }


@router.delete("/queue/{order_id}")
def remove_from_queue(
    order_id: int,
    db: Session = Depends(get_db)
):
    """Remove a specific order from the robot queue"""
    status = robot_service.get_queue_status(db, order_id)
    
    if not status:
        raise HTTPException(
            status_code=404,
            detail=f"Order {order_id} not found in queue"
        )
    
    # Delete the queue entry
    db.query(RobotQueue).filter(RobotQueue.order_id == order_id).delete()
    db.commit()
    
    return {
        "message": f"Order {order_id} removed from queue",
        "order_id": order_id
    }


@router.post("/queue/clear")
def clear_queue(db: Session = Depends(get_db)):
    """Clear all orders from the robot queue"""
    count = db.query(RobotQueue).count()
    db.query(RobotQueue).delete()
    db.commit()
    
    return {
        "message": f"Cleared {count} orders from queue",
        "cleared_count": count
    }
