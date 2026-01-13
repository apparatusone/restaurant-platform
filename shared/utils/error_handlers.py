"""
Shared error handling utilities for all microservices.
"""

import logging
from typing import Optional, Dict, Any
from fastapi import HTTPException, status
from sqlalchemy.exc import (
    SQLAlchemyError, 
    IntegrityError, 
    OperationalError,
    DataError
)

logger = logging.getLogger(__name__)

def handle_database_error(
    e: SQLAlchemyError,
    operation: str,
    log_context: Optional[Dict[str, Any]] = None,
    user_message: Optional[str] = None
) -> HTTPException:
    """
    Safely handle database errors without exposing internals.
    
    Args:
        e: SQLAlchemy exception
        operation: Description of what was being attempted (for logs)
        log_context: Additional context to log (e.g., {"user_id": 123})
        user_message: Optional custom message for the user
    
    Returns:
        HTTPException with safe error message
    """
    # log full details server-side
    logger.error(
        f"Database error during {operation}",
        extra=log_context or {},
        exc_info=True
    )
    
    if isinstance(e, IntegrityError):
        return HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=user_message or "A database constraint was violated"
        )
    elif isinstance(e, OperationalError):
        return HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=user_message or "Database temporarily unavailable"
        )
    elif isinstance(e, DataError):
        return HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=user_message or "Invalid data provided"
        )
    
    # generic error
    return HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=user_message or "A database error occurred"
    )


def handle_generic_error(
    e: Exception,
    operation: str,
    log_context: Optional[Dict[str, Any]] = None,
    status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR
) -> HTTPException:
    """

    Args:
        e: exception
        operation: Description of what was being attempted
        log_context: Additional context to log
        status_code: HTTP status code to return
    
    Returns:
        HTTPException with generic error message
    """
    logger.error(
        f"Error during {operation}: {type(e).__name__}",
        extra=log_context or {},
        exc_info=True
    )
    
    return HTTPException(
        status_code=status_code,
        detail="An error occurred while processing your request"
    )


def create_global_exception_handler(service_name: str):
    """
    Factory function to create a global exception handler for a service.
    
    Usage in main.py:
        @app.exception_handler(Exception)
        async def handler(request, exc):
            return await create_global_exception_handler("order-service")(request, exc)
    """
    async def global_exception_handler(request, exc: Exception):
        from fastapi.responses import JSONResponse
        
        # Log full details server-side
        logger.error(
            f"[{service_name}] Unhandled exception: {request.method} {request.url.path}",
            extra={
                "service": service_name,
                "method": request.method,
                "path": request.url.path,
                "exception_type": type(exc).__name__
            },
            exc_info=True
        )
        
        # Return generic error
        return JSONResponse(
            status_code=500,
            content={"detail": "An internal server error occurred"},
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Credentials": "true",
                "Access-Control-Allow-Methods": "*",
                "Access-Control-Allow-Headers": "*",
            }
        )
    
    return global_exception_handler