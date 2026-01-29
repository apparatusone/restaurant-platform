import json
import logging
from datetime import datetime
from typing import Any, Dict, Iterable, Optional

import redis

from ..schemas.orders import IngredientStatus

logger = logging.getLogger(__name__)

_redis_client: Optional[redis.Redis] = None


def set_client(client: Optional[redis.Redis]):
    global _redis_client
    _redis_client = client


def get_client() -> redis.Redis:
    if _redis_client is None:
        raise RuntimeError("Redis client not initialized")
    return _redis_client


def close_client():
    global _redis_client
    if _redis_client is None:
        return
    try:
        _redis_client.close()
    finally:
        _redis_client = None


def get_order_key(order_id: int) -> str:
    return f"order:{order_id}"


def get_order_queue_key(order_id: int) -> str:
    return f"order:{order_id}:queue"


def save_order_state(order_id: int, state: Dict[str, Any]):
    client = get_client()
    key = get_order_key(order_id)
    state["updated_at"] = datetime.utcnow().isoformat()
    client.set(key, json.dumps(state))


def get_order_state(order_id: int) -> Optional[Dict[str, Any]]:
    client = get_client()
    key = get_order_key(order_id)
    data = client.get(key)
    if data:
        return json.loads(data)
    return None


def push_ingredient_to_queue(order_id: int, ingredient_name: str, apriltag_id: int):
    client = get_client()
    queue_key = get_order_queue_key(order_id)
    item = {
        "ingredient": ingredient_name,
        "apriltag_id": apriltag_id,
        "status": IngredientStatus.PENDING,
    }
    client.rpush(queue_key, json.dumps(item))


def pop_ingredient_from_queue(order_id: int) -> Optional[Dict[str, Any]]:
    client = get_client()
    queue_key = get_order_queue_key(order_id)
    data = client.lpop(queue_key)
    if data:
        return json.loads(data)
    return None


def get_queue_length(order_id: int) -> int:
    client = get_client()
    queue_key = get_order_queue_key(order_id)
    return int(client.llen(queue_key))


def get_robot_enabled_state() -> bool:
    client = get_client()
    enabled = client.get("robot_kitchen_enabled")
    return enabled == "true" if enabled else False


def set_robot_enabled_state(enabled: bool) -> bool:
    client = get_client()
    client.set("robot_kitchen_enabled", "true" if enabled else "false")
    return enabled


def scan_order_keys() -> Iterable[str]:
    client = get_client()
    return client.scan_iter("order:*")


def delete_key(key: str):
    client = get_client()
    client.delete(key)


def delete_order(order_id: int):
    client = get_client()
    client.delete(get_order_key(order_id))
    client.delete(get_order_queue_key(order_id))
