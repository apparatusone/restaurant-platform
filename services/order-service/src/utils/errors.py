"""
Standardized error handling utilities for the API
"""
from datetime import datetime, timezone
from typing import Dict, Any, List, Optional
from fastapi import HTTPException, status
from sqlalchemy.exc import SQLAlchemyError


class APIError:
    """Standard error response format"""
    
    def __init__(
        self,
        detail: str,
        error_code: str,
        status_code: int = status.HTTP_400_BAD_REQUEST,
        extra_data: Optional[Dict[str, Any]] = None
    ):
        self.detail = detail
        self.error_code = error_code
        self.status_code = status_code
        self.extra_data = extra_data or {}
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert error to dict"""
        error_dict = {
            "detail": self.detail,
            "error_code": self.error_code,
            "timestamp": datetime.now(timezone.utc).isoformat() + "Z"
        }
        error_dict.update(self.extra_data)
        return error_dict
    
    def raise_exception(self):
        """Raise HTTPException with standardized format"""
        raise HTTPException(
            status_code=self.status_code,
            detail=self.to_dict()
        )


class NotFoundError(APIError):
    def __init__(self, resource: str, identifier: Any):
        super().__init__(
            detail=f"{resource} with identifier '{identifier}' not found",
            error_code="NOT_FOUND",
            status_code=status.HTTP_404_NOT_FOUND
        )


class ValidationError(APIError):
    def __init__(self, detail: str, extra_data: Optional[Dict[str, Any]] = None):
        super().__init__(
            detail=detail,
            error_code="VALIDATION_ERROR",
            status_code=status.HTTP_400_BAD_REQUEST,
            extra_data=extra_data
        )


class ConflictError(APIError):
    def __init__(self, detail: str):
        super().__init__(
            detail=detail,
            error_code="CONFLICT",
            status_code=status.HTTP_409_CONFLICT
        )


class DatabaseError(APIError):
    def __init__(self, detail: str = "Database operation failed"):
        super().__init__(
            detail=detail,
            error_code="DATABASE_ERROR",
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR
        )


class StatusTransitionError(APIError):
    def __init__(
        self,
        current_status: str,
        requested_status: str,
        valid_next_states: List[str]
    ):
        super().__init__(
            detail="Invalid status transition",
            error_code="INVALID_STATUS_TRANSITION",
            status_code=status.HTTP_400_BAD_REQUEST,
            extra_data={
                "current_status": current_status,
                "requested_status": requested_status,
                "valid_next_states": valid_next_states
            }
        )


def handle_sqlalchemy_error(e: SQLAlchemyError) -> APIError:
    """Convert SQLAlchemy errors to standardized format"""
    error_msg = str(e.__dict__.get('orig', str(e)))
    
    if "UNIQUE constraint failed" in error_msg or "duplicate key" in error_msg.lower():
        return ConflictError("Resource already exists")
    elif "FOREIGN KEY constraint failed" in error_msg or "foreign key" in error_msg.lower():
        return ValidationError("Referenced resource does not exist")
    else:
        return DatabaseError(f"Database error: {error_msg}")


# convenience function
def raise_not_found(resource: str, identifier: Any):
    NotFoundError(resource, identifier).raise_exception()


def raise_validation_error(detail: str, extra_data: Optional[Dict[str, Any]] = None):
    ValidationError(detail, extra_data).raise_exception()


def raise_conflict_error(detail: str):
    ConflictError(detail).raise_exception()


def raise_status_transition_error(
    current_status: str,
    requested_status: str,
    valid_next_states: List[str]
):
    StatusTransitionError(
        current_status,
        requested_status,
        valid_next_states
    ).raise_exception()