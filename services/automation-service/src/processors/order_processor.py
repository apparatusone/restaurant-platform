import logging
from typing import Optional

import httpx

from ..clients import robot_client
from ..schemas.orders import OrderStatus
from ..services import redis_service

logger = logging.getLogger(__name__)


async def notify_callback(
    callback_url: str,
    order_id: int,
    status: str,
    error_message: Optional[str] = None,
):
    if not callback_url:
        return

    try:
        payload = {
            "order_id": order_id,
            "status": status,
        }
        if error_message:
            payload["error_message"] = error_message

        async with httpx.AsyncClient(timeout=10.0) as client:
            await client.post(callback_url, json=payload)
    except Exception:
        logger.warning("Failed to notify callback", extra={"order_id": order_id, "callback_url": callback_url}, exc_info=True)


async def process_order_task(order_id: int):
    state = redis_service.get_order_state(order_id)
    if not state:
        return

    if not redis_service.get_robot_enabled_state():
        state["status"] = OrderStatus.CANCELLED
        state["error_message"] = "robot kitchen integration is disabled"
        redis_service.save_order_state(order_id, state)

        callback_url = state.get("callback_url")
        if callback_url:
            await notify_callback(callback_url, order_id, state["status"], state["error_message"])
        return

    try:
        robot_status = await robot_client.get_robot_status()

        if not robot_status.get("ready", False):
            state["status"] = OrderStatus.FAILED
            reasons = []
            if not robot_status.get("homed", False):
                reasons.append("robot not homed")
            if not robot_status.get("calibrated", False):
                reasons.append("camera not calibrated")
            state["error_message"] = ", ".join(reasons) if reasons else "robot not ready"
            redis_service.save_order_state(order_id, state)

            callback_url = state.get("callback_url")
            if callback_url:
                await notify_callback(callback_url, order_id, state["status"], state["error_message"])
            return
    except Exception as e:
        state["status"] = OrderStatus.FAILED
        state["error_message"] = f"failed to check robot status: {str(e)}"
        redis_service.save_order_state(order_id, state)

        callback_url = state.get("callback_url")
        if callback_url:
            await notify_callback(callback_url, order_id, state["status"], state["error_message"])
        return

    state["status"] = OrderStatus.PROCESSING
    redis_service.save_order_state(order_id, state)

    while True:
        ingredient_item = redis_service.pop_ingredient_from_queue(order_id)
        if not ingredient_item:
            break

        ingredient_name = ingredient_item["ingredient"]
        apriltag_id = ingredient_item["apriltag_id"]

        state["current_ingredient"] = ingredient_name
        redis_service.save_order_state(order_id, state)

        result = await robot_client.send_pick_to_robot(apriltag_id)

        if result.get("status") == "success":
            state["completed_ingredients"] += 1
        else:
            state["failed_ingredients"] += 1
            state["error_message"] = result.get("reason", "unknown error")

        redis_service.save_order_state(order_id, state)

    await robot_client.send_robot_home()

    if state["failed_ingredients"] > 0:
        state["status"] = OrderStatus.FAILED
    else:
        state["status"] = OrderStatus.COMPLETED

    redis_service.save_order_state(order_id, state)

    callback_url = state.get("callback_url")
    if callback_url:
        await notify_callback(
            callback_url,
            order_id,
            state["status"],
            state.get("error_message"),
        )
