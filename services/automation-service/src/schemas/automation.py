from typing import Optional

from pydantic import BaseModel


class AutomationStatusResponse(BaseModel):
    is_busy: bool
    status: str
    message: str
    current_order_id: Optional[int] = None
    current_ingredient: Optional[str] = None
    homed: bool = False
    calibrated: bool = False
    ready: bool = False
    error: Optional[str] = None


class RobotKitchenEnabledResponse(BaseModel):
    robot_kitchen_enabled: bool


class QueueOrderItem(BaseModel):
    order_id: int
    status: str
    total_ingredients: int = 0
    completed_ingredients: int = 0
    failed_ingredients: int = 0
    current_ingredient: Optional[str] = None
    error_message: Optional[str] = None
    retry_count: int = 0
    created_at: Optional[str] = None
    updated_at: Optional[str] = None
