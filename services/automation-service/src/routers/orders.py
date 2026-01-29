from datetime import datetime

from fastapi import APIRouter, BackgroundTasks, HTTPException

from ..processors.order_processor import process_order_task
from ..schemas.orders import OrderStatus, OrderStatusResponse, ProcessOrderRequest
from ..services import redis_service

router = APIRouter()


@router.post("/orders/process")
async def process_order(request: ProcessOrderRequest, background_tasks: BackgroundTasks):
    if not redis_service.get_robot_enabled_state():
        raise HTTPException(status_code=400, detail="Robot kitchen integration is disabled")

    existing = redis_service.get_order_state(request.order_id)
    if existing and existing["status"] in [OrderStatus.PROCESSING, OrderStatus.QUEUED]:
        raise HTTPException(status_code=409, detail=f"Order {request.order_id} is already being processed")

    seen_apriltag_ids = set()
    unique_ingredients = []
    for item in request.ingredients:
        if item.apriltag_id not in seen_apriltag_ids:
            seen_apriltag_ids.add(item.apriltag_id)
            unique_ingredients.append(item)

    total_picks = len(unique_ingredients)
    for item in unique_ingredients:
        redis_service.push_ingredient_to_queue(request.order_id, item.name, item.apriltag_id)

    state = {
        "order_id": request.order_id,
        "status": OrderStatus.QUEUED,
        "total_ingredients": total_picks,
        "completed_ingredients": 0,
        "failed_ingredients": 0,
        "current_ingredient": None,
        "error_message": None,
        "callback_url": request.callback_url,
        "created_at": datetime.utcnow().isoformat(),
        "updated_at": datetime.utcnow().isoformat(),
    }
    redis_service.save_order_state(request.order_id, state)

    background_tasks.add_task(process_order_task, request.order_id)

    return {
        "order_id": request.order_id,
        "status": "queued",
        "total_ingredients": total_picks,
        "message": "Order queued for processing",
    }


@router.get("/orders/{order_id}/status", response_model=OrderStatusResponse)
async def get_order_status(order_id: int):
    state = redis_service.get_order_state(order_id)
    if not state:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")

    return OrderStatusResponse(**state)


@router.post("/orders/{order_id}/cancel")
async def cancel_order(order_id: int):
    state = redis_service.get_order_state(order_id)
    if not state:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")

    if state["status"] == OrderStatus.PROCESSING:
        raise HTTPException(status_code=409, detail="Cannot cancel order that is currently processing")

    if state["status"] in [OrderStatus.COMPLETED, OrderStatus.FAILED, OrderStatus.CANCELLED]:
        raise HTTPException(status_code=409, detail=f"Order already {state['status']}")

    redis_service.delete_key(redis_service.get_order_queue_key(order_id))
    state["status"] = OrderStatus.CANCELLED
    redis_service.save_order_state(order_id, state)

    return {"order_id": order_id, "status": "cancelled"}
