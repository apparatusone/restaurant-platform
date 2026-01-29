import logging
from typing import Optional

import redis

from shared.dependencies.config import conf

from ..services import redis_service

logger = logging.getLogger(__name__)


def index():
    client: Optional[redis.Redis] = None

    try:
        client = redis.Redis(host=conf.redis_host, port=conf.redis_port, decode_responses=True)
        client.ping()
        logger.info("Connected to Redis", extra={"redis_host": conf.redis_host, "redis_port": conf.redis_port})
    except Exception:
        logger.error("Failed to connect to Redis", extra={"redis_host": conf.redis_host, "redis_port": conf.redis_port}, exc_info=True)

    redis_service.set_client(client)


def shutdown():
    redis_service.close_client()
