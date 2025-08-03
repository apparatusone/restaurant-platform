from sqlalchemy.orm import Session
from enum import Enum

class SortOptions(str, Enum):
    RATING = "rating"
    POPULARITY = "popularity"
    NEWEST_REVIEW = "newest review"
    OLDES_REVIEW = "oldest review"


class TimeRange(str, Enum):
    WEEK = "week"
    MONTH = "month"
    YEAR = "year"
    ALL_TIME = "all time"


def filter_by_time_range(query, time_range: TimeRange, date_field):
    """
    Filter the by the time range
    """
    from datetime import datetime, timedelta
    
    if time_range == TimeRange.WEEK:
        start_date = datetime.now() - timedelta(days=7)
        return query.filter(date_field >= start_date)
    elif time_range == TimeRange.MONTH:
        start_date = datetime.now() - timedelta(days=30)
        return query.filter(date_field >= start_date)
    elif time_range == TimeRange.YEAR:
        start_date = datetime.now() - timedelta(days=365)
        return query.filter(date_field >= start_date)
    elif time_range == TimeRange.ALL_TIME:
        return query  # No filter for all time


def get_dish_analytics_average_rating(db: Session, time_range: TimeRange = TimeRange.WEEK):
    """
    Get list of dishes with their average ratings
    """
    from sqlalchemy import func, cast, Float
    from ..models.menu_items import MenuItem
    from ..models.reviews import Reviews

    # get the filtered reviews subquery
    reviews_query = db.query(Reviews)
    filtered_reviews = filter_by_time_range(reviews_query, time_range, Reviews.created_at).subquery()
    
    # calc average rating
    avg_rating = func.avg(cast(filtered_reviews.c.rating, Float)).label("avg_rating")
    
    rows = (
        db.query(
            MenuItem.name.label("dish_name"),
            avg_rating,
        )
        .outerjoin(filtered_reviews, MenuItem.id == filtered_reviews.c.menu_item_id)
        .group_by(MenuItem.id, MenuItem.name)
        .order_by(MenuItem.name)
        .all()
    )

    return [
        {
            "dish_name": dish_name,
            "average_rating": (round(float(avg), 1) if avg is not None else "No ratings")
        }
        for dish_name, avg in rows
    ]


# get review, with item name, sort by  (newest or oldest)
# filter by rating (1-5, all)
