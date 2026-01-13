"""
Centralized logging configuration for all microservices.
"""
import logging
import sys
from typing import Optional


def setup_logging(
    service_name: str,
    level: str = "INFO",
    format_string: Optional[str] = None
):
    """
    Configure structured logging for a microservice.
    
    Args:
        service_name: Name of the service
        level: Logging level (DEBUG, INFO, WARNING, ERROR)
        format_string: Custom format string (optional)
    """
    if format_string is None:
        format_string = (
            f'%(asctime)s - [{service_name}] - '
            f'%(name)s - %(levelname)s - %(message)s'
        )
    
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format=format_string,
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    # Reduce logging from third-party libraries
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)