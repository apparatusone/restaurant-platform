from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from shared.dependencies.database import get_db
from ..services.analytics import ValueSort
from ..services.analytics import TimeSort
from ..services.analytics import TimeRange
from ..services import analytics

router = APIRouter(
    tags=['Analytics'],
    prefix="/analytics"
)


@router.get("/dish-analytics-average-rating")
def get_dish_analytics_average_rating(
    time_range: TimeRange = TimeRange.WEEK,
    db: Session = Depends(get_db)
):
    """
    Get a list of menu items and their average rating in the selected time range
    """
    return analytics.get_dish_analytics_average_rating(db=db, time_range=time_range)


@router.get("/dish-analytics-popularity")
def get_dish_analytics_popularity(
    time_range: TimeRange = TimeRange.WEEK,
    sort_by: ValueSort = ValueSort.LOW,
    db: Session = Depends(get_db)
):
    """
    Get a list of menu items and their order count in the selected time range
    """
    return analytics.get_dish_analytics_popularity(db=db, time_range=time_range, sort_by=sort_by)


@router.get("/view-reviews")
def view_reviews(sort_by: TimeSort, db: Session = Depends(get_db)):
    """
    Get all reviews sorted by date
    """
    return analytics.view_reviews(db=db, sort_by=sort_by)