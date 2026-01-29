from enum import Enum
from typing import List, Optional

from pydantic import BaseModel


class OrderStatus(str, Enum):
    QUEUED = "queued"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class IngredientStatus(str, Enum):
    PENDING = "pending"
    PICKING = "picking"
    COMPLETED = "completed"
    FAILED = "failed"


class IngredientItem(BaseModel):
    name: str
    apriltag_id: int


class ProcessOrderRequest(BaseModel):
    order_id: int
    ingredients: List[IngredientItem]
    callback_url: Optional[str] = None


class OrderStatusResponse(BaseModel):
    order_id: int
    status: OrderStatus
    total_ingredients: int
    completed_ingredients: int
    failed_ingredients: int
    current_ingredient: Optional[str] = None
    error_message: Optional[str] = None
    created_at: str
    updated_at: str
