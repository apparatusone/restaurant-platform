from sqlalchemy.orm import Session
from datetime import datetime
from enum import Enum

class ValueSort(str, Enum):
    LOW = "low"
    HIGH = "high"


class TimeSort(str, Enum):
    NEWEST = "newest"
    OLDEST = "oldest"


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

    # get the filtered reviews
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

def get_dish_analytics_popularity(db: Session, 
                                  time_range: TimeRange = TimeRange.WEEK, 
                                  sort_by: ValueSort = ValueSort.LOW
                                  ):
    """
    Get a list of menu items and their order count in the selected time range
    """
    from sqlalchemy import func
    from ..models.menu_items import MenuItem
    from ..models.orders import Order
    from ..models.order_details import OrderDetail
    
    # sum total quantity
    order_count = func.sum(OrderDetail.amount).label("order_count")
    
    query = (
        db.query(
            MenuItem.name.label("dish_name"),
            order_count,
        )
        .outerjoin(OrderDetail, MenuItem.id == OrderDetail.menu_item_id)
        .outerjoin(Order, OrderDetail.order_id == Order.id)
        .group_by(MenuItem.id, MenuItem.name)
    )
    
    # time filter
    query = filter_by_time_range(query, time_range, Order.order_date)
    
    # sort
    if sort_by == ValueSort.LOW:
        query = query.order_by(order_count)
    elif sort_by == ValueSort.HIGH:
        query = query.order_by(order_count.desc())
    
    rows = query.all()
    
    return [
        {
            "dish_name": dish_name,
            "order_count": int(count) if count is not None else 0
        }
        for dish_name, count in rows
    ]

def view_reviews(db: Session, sort_by: TimeSort):
    """
    Get all reviews sorted by date
    """
    from ..models.reviews import Reviews
    from ..models.menu_items import MenuItem
    
    # get all reviews with menu item information
    query = db.query(Reviews).join(MenuItem, Reviews.menu_item_id == MenuItem.id)
    
    # sort by date
    if sort_by == TimeSort.NEWEST:
        query = query.order_by(Reviews.created_at)
    else:
        query = query.order_by(Reviews.created_at.desc())
    
    reviews = query.all()

    return [
        {
            "menu_item_name": review.menu_item.name,
            "rating": review.rating,
            "text": review.review_text,
            "date": review.created_at
        }
        for review in reviews
    ]


# get review, with item name, sort by  (newest or oldest)
# filter by rating (1-5, all)
