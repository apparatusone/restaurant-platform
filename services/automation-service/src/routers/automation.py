import logging
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, BackgroundTasks, HTTPException

from ..clients import robot_client
from ..schemas.automation import AutomationStatusResponse
from ..services import redis_service
from ..schemas.orders import OrderStatus
from ..processors.order_processor import process_order_task

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/")
async def root():
    return {"service": "automation-service", "version": "1.0.0"}


@router.get("/health")
async def health():
    try:
        redis_service.get_client().ping()
        redis_status = "connected"
    except Exception:
        redis_status = "disconnected"

    return {
        "status": "healthy",
        "redis": redis_status,
    }


@router.get("/automation/status", response_model=AutomationStatusResponse)
async def get_status():
    robot_status: Dict[str, Any] = {
        "homed": False,
        "calibrated": False,
        "ready": False,
    }

    try:
        robot_status = await robot_client.get_robot_status()
    except Exception as e:
        robot_status["error"] = str(e)

    is_busy = False
    current_order: Optional[int] = None
    current_ingredient: Optional[str] = None
    status_message = "Idle"

    for key in redis_service.scan_order_keys():
        if ":queue" in key:
            continue
        try:
            order_id = int(key.split(":")[1])
            state = redis_service.get_order_state(order_id)
            if state and state.get("status") == OrderStatus.PROCESSING:
                is_busy = True
                current_order = state.get("order_id")
                current_ingredient = state.get("current_ingredient")
                status_message = f"Processing order #{current_order}"
                if current_ingredient:
                    status_message += f" - picking {current_ingredient}"
                break
        except (ValueError, IndexError):
            continue

    return AutomationStatusResponse(
        is_busy=is_busy,
        status=status_message,
        message=status_message,
        current_order_id=current_order,
        current_ingredient=current_ingredient,
        homed=bool(robot_status.get("homed", False)),
        calibrated=bool(robot_status.get("calibrated", False)),
        ready=bool(robot_status.get("ready", False)),
        error=robot_status.get("error"),
    )


@router.get("/automation/queue")
async def get_queue() -> List[Dict[str, Any]]:
    orders: List[Dict[str, Any]] = []

    for key in redis_service.scan_order_keys():
        if ":queue" in key:
            continue
        try:
            order_id = int(key.split(":")[1])
            state = redis_service.get_order_state(order_id)
            if state:
                orders.append(
                    {
                        "order_id": state.get("order_id"),
                        "status": state.get("status"),
                        "total_ingredients": state.get("total_ingredients", 0),
                        "completed_ingredients": state.get("completed_ingredients", 0),
                        "failed_ingredients": state.get("failed_ingredients", 0),
                        "current_ingredient": state.get("current_ingredient"),
                        "error_message": state.get("error_message"),
                        "retry_count": state.get("retry_count", 0),
                        "created_at": state.get("created_at"),
                        "updated_at": state.get("updated_at"),
                    }
                )
        except (ValueError, IndexError):
            continue

    orders.sort(key=lambda x: x.get("created_at", ""), reverse=True)
    return orders


@router.get("/automation/enabled")
async def get_enabled():
    return {"robot_kitchen_enabled": redis_service.get_robot_enabled_state()}


@router.post("/automation/enabled")
async def set_enabled(enabled: bool):
    value = redis_service.set_robot_enabled_state(enabled)
    return {"robot_kitchen_enabled": value}


@router.post("/automation/queue/process-pending")
async def process_pending(background_tasks: BackgroundTasks):
    if not redis_service.get_robot_enabled_state():
        return {"processed_orders": [], "count": 0}

    processed: List[int] = []

    for key in redis_service.scan_order_keys():
        if ":queue" in key:
            continue
        try:
            order_id = int(key.split(":")[1])
            state = redis_service.get_order_state(order_id)
            if state and state.get("status") == OrderStatus.QUEUED:
                background_tasks.add_task(process_order_task, order_id)
                processed.append(order_id)
        except (ValueError, IndexError):
            continue

    return {"processed_orders": processed, "count": len(processed)}


@router.post("/automation/queue/clear")
async def clear_queue():
    cleared = 0

    for key in redis_service.scan_order_keys():
        redis_service.delete_key(key)
        cleared += 1

    return {"cleared": cleared}


@router.delete("/automation/queue/{order_id}")
async def remove_from_queue(order_id: int):
    state = redis_service.get_order_state(order_id)
    if not state:
        raise HTTPException(status_code=404, detail=f"Order {order_id} not found")

    if state["status"] == OrderStatus.PROCESSING:
        raise HTTPException(status_code=409, detail="Cannot remove order that is currently processing")

    redis_service.delete_order(order_id)

    return {"order_id": order_id, "status": "removed"}
