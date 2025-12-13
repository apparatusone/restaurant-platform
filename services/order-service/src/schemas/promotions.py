from datetime import datetime
from pydantic import BaseModel, Field
from typing import Optional


class PromotionBase(BaseModel):
    code: str
    description: Optional[str] = None
    discount_percent: int = Field(..., ge=1, le=100, description="Percentage from 1 to 100")
    expiration_date: Optional[datetime] = None


class PromotionCreate(PromotionBase):
    pass

class PromotionUpdate(BaseModel):
    discount_percent: int = Field(..., ge=1, le=100, description="Percentage from 1 to 100")
    expiration_date: Optional[datetime] = None

class Promotion(PromotionBase):
    id: int
    created_at: Optional[datetime] = None

    class ConfigDict:
        from_attributes = True