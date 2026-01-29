import json
import logging
import redis
import os
import asyncio
from datetime import datetime
from sqlalchemy import text
from shared.dependencies.database import SessionLocal
from ..clients.robot_client import send_pick_to_robot, send_robot_home, get_robot_status
from ..services.redis_service import (
    save_order_state, get_order_state, push_ingredient_to_queue,
    pop_ingredient_from_queue, get_robot_enabled_state
)
from ..schemas.orders import OrderStatus

logger = logging.getLogger(__name__)

redis_client = redis.Redis(
    host=os.getenv("REDIS_HOST", "localhost"),
    port=int(os.getenv("REDIS_PORT", 6379)),
    decode_responses=True
)


async def process_kitchen_order(check_id: int, item_ids: list[int]):
    """Process kitchen order using Redis queue"""
    
    if not get_robot_enabled_state():
        logger.warning(f"Robot disabled, skipping check {check_id}")
        return
    
    db = SessionLocal()
    try:
        # ONLY get ingredients with AprilTags
        # TODO: should this be a direct DB query?
        query = text("""
            SELECT DISTINCT
                i.id as ingredient_id,
                i.name as ingredient_name,
                i.apriltag_id
            FROM check_items ci
            JOIN menu_items mi ON ci.menu_item_id = mi.id
            JOIN recipes r ON mi.id = r.menu_item_id
            JOIN ingredients i ON r.ingredient_id = i.id
            WHERE ci.id IN :item_ids
            AND i.apriltag_id IS NOT NULL
        """)
        
        results = db.execute(query, {"item_ids": tuple(item_ids)}).fetchall()
        
        if not results:
            logger.warning(f"No ingredients with AprilTags found for check {check_id}")
            return
        
        # Queue ingredients
        for row in results:
            push_ingredient_to_queue(check_id, row.ingredient_name, row.apriltag_id)
        
        # Initialize order state
        state = {
            "order_id": check_id,
            "status": OrderStatus.QUEUED,
            "total_ingredients": len(results),
            "completed_ingredients": 0,
            "failed_ingredients": 0,
            "current_ingredient": None,
            "error_message": None,
            "created_at": datetime.utcnow().isoformat(),
            "updated_at": datetime.utcnow().isoformat()
        }
        save_order_state(check_id, state)
        
        logger.info(f"Queued check {check_id} with {len(results)} ingredients")
        
        # Check robot status
        try:
            robot_status = await get_robot_status()
            if not robot_status.get('ready', False):
                state['status'] = OrderStatus.FAILED
                state['error_message'] = 'robot not ready'
                save_order_state(check_id, state)
                return
        except Exception as e:
            state['status'] = OrderStatus.FAILED
            state['error_message'] = f'failed to check robot: {str(e)}'
            save_order_state(check_id, state)
            return
        
        # Process queue
        state['status'] = OrderStatus.PROCESSING
        save_order_state(check_id, state)
        
        while True:
            item = pop_ingredient_from_queue(check_id)
            if not item:
                break
            
            state['current_ingredient'] = item['ingredient']
            save_order_state(check_id, state)
            
            logger.info(f"Picking {item['ingredient']} (AprilTag {item['apriltag_id']})")
            result = await send_pick_to_robot(item['apriltag_id'])
            
            if result.get('status') == 'success':
                state['completed_ingredients'] += 1
            else:
                state['failed_ingredients'] += 1
                state['error_message'] = result.get('reason', 'unknown')
            
            save_order_state(check_id, state)
        
        # Done - send robot home
        await send_robot_home()
        
        state['status'] = OrderStatus.COMPLETED if state['failed_ingredients'] == 0 else OrderStatus.FAILED
        state['current_ingredient'] = None
        save_order_state(check_id, state)
        
        logger.info(f"Completed check {check_id}")
        
    except Exception as e:
        logger.error(f"Error processing kitchen order: {e}", exc_info=True)
    finally:
        db.close()


def start_subscriber():
    """Subscribe to kitchen orders channel"""
    pubsub = redis_client.pubsub()
    pubsub.subscribe("kitchen.orders")
    
    logger.info("Subscribed to kitchen.orders channel")
    
    for message in pubsub.listen():
        if message["type"] == "message":
            try:
                data = json.loads(message["data"])
                check_id = data["check_id"]
                item_ids = data["item_ids"]
                logger.info(f"Received kitchen order: check {check_id}, items {item_ids}")
                asyncio.run(process_kitchen_order(check_id, item_ids))
            except Exception as e:
                logger.error(f"Error handling message: {e}", exc_info=True)
