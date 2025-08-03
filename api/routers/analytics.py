from fastapi import APIRouter, Depends, FastAPI, status, Response
from sqlalchemy.orm import Session
from ..dependencies.database import get_db
from ..services.analytics import SortOptions
from ..services.analytics import TimeRange
from ..services import analytics

router = APIRouter(
    tags=['Analytics'],
    prefix="/analytics"
)


@router.get("/dish-analytics-average-rating")
def get_dish_analytics(
    time_range: TimeRange = TimeRange.WEEK,
    db: Session = Depends(get_db)
):
    """
    Get a list of menu items and their average rating
    """
    return analytics.get_dish_analytics_average_rating(db=db, time_range=time_range)