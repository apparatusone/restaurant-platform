import json
import logging
import redis
import os
import asyncio
from datetime import datetime
from ..clients.robot_client import send_pick_to_robot, send_robot_home, get_robot_status
from ..services.redis_service import (
    save_order_state, push_ingredient_to_queue,
    pop_ingredient_from_queue, get_robot_enabled_state, RESTAURANT_ID
)
from ..schemas.orders import OrderStatus

logger = logging.getLogger(__name__)

redis_client = redis.Redis(
    host=os.getenv("REDIS_HOST", "localhost"),
    port=int(os.getenv("REDIS_PORT", 6379)),
    decode_responses=True
)


async def process_kitchen_order(check_id: int, ingredients: list[dict]):
    """Process kitchen order - ingredients already resolved by order-service"""
    
    if not get_robot_enabled_state():
        logger.warning(f"Robot disabled, skipping check {check_id}")
        return
    
    if not ingredients:
        logger.warning(f"No ingredients for check {check_id}")
        return
    
    # Queue ingredients
    for ing in ingredients:
        push_ingredient_to_queue(check_id, ing["name"], ing["apriltag_id"])
    
    # Initialize order state
    state = {
        "order_id": check_id,
        "status": OrderStatus.QUEUED,
        "total_ingredients": len(ingredients),
        "completed_ingredients": 0,
        "failed_ingredients": 0,
        "current_ingredient": None,
        "error_message": None,
        "created_at": datetime.utcnow().isoformat(),
        "updated_at": datetime.utcnow().isoformat()
    }
    save_order_state(check_id, state)
    
    logger.info(f"Queued check {check_id} with {len(ingredients)} ingredients")
    
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


def start_subscriber():
    """Subscribe to kitchen orders channel"""
    pubsub = redis_client.pubsub()
    
    channel = f"kitchen.orders:{RESTAURANT_ID}" if RESTAURANT_ID else "kitchen.orders"
    pubsub.subscribe(channel)
    
    logger.info(f"Subscribed to {channel}")
    
    for message in pubsub.listen():
        if message["type"] == "message":
            try:
                data = json.loads(message["data"])
                check_id = data["check_id"]
                ingredients = data["ingredients"]
                logger.info(f"Received kitchen order: check {check_id}, {len(ingredients)} ingredients")
                asyncio.run(process_kitchen_order(check_id, ingredients))
            except Exception as e:
                logger.error(f"Error handling message: {e}", exc_info=True)
