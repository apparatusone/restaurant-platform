import logging
from typing import Any, Dict

import httpx

from shared.dependencies.config import conf
from shared.utils.http_client import CircuitBreakerOpenError, ResilientHttpClient

logger = logging.getLogger(__name__)

_robot_client = ResilientHttpClient(
    base_url=conf.ros2_robot_url,
    max_retries=3,
    timeout=120.0,
)


async def get_robot_status() -> Dict[str, Any]:
    response = await _robot_client.get("/robot/status")
    return response.json()


async def send_pick_to_robot(cube_id: int) -> Dict[str, Any]:
    try:
        response = await _robot_client.post(
            "/robot/pick_cube",
            json={"cube_apriltag_id": cube_id},
        )
        return response.json()
    except CircuitBreakerOpenError:
        return {"status": "error", "reason": "circuit_breaker_open"}
    except httpx.TimeoutException:
        return {"status": "error", "reason": "timeout"}
    except httpx.HTTPError as e:
        return {"status": "error", "reason": f"http_error: {str(e)}"}
    except Exception as e:
        return {"status": "error", "reason": f"unknown: {str(e)}"}


async def send_robot_home() -> Dict[str, Any]:
    try:
        response = await _robot_client.post("/robot/home")
        return response.json()
    except Exception as e:
        logger.warning("Failed to send robot home", exc_info=True)
        return {"status": "error", "reason": str(e)}
