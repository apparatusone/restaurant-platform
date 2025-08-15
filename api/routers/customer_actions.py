from fastapi import APIRouter, Depends, Query
from sqlalchemy.orm import Session
from typing import Optional, Annotated
from ..services import customer as customer_services

from ..schemas import order_details as order_detail_schema
from ..schemas import payment_method as payment_schema
from ..schemas import customers as customer_schema
from ..schemas import orders as order_schema
from ..schemas import reviews as review_schema
from ..schemas import order_submission as order_submission_schema
from ..controllers import payment_method as payment_controller
from ..controllers import customers as customer_controller
from ..controllers import orders as order_controller
from ..controllers import reviews as review_controller
from ..dependencies.database import get_db
from ..schemas.orders import OrderType

router = APIRouter(
    tags=['Customer Actions'],
    prefix="/customer"
)


@router.get("/track-order/{tracking_number}")
def get_tracking_information(tracking_number: str, db: Session = Depends(get_db)):
    """
    Get order tracking information using tracking number
    """
    return customer_services.get_tracking_information(
        db=db,
        tracking_number=tracking_number
    )


@router.post("/review-dish", response_model=review_schema.Reviews)
def add_review_to_menu_item(
    menu_item_id: int,
    customer_name: str,
    rating: Annotated[int, Query(ge=1, le=5, description="Rating from 1 to 5 stars")],
    review_text: Annotated[Optional[str], Query(description="Optional review comments")] = None,
    db: Session = Depends(get_db)
):
    """
    Add customer review to menu item
    """
    # Create the request object for the controller
    request = review_schema.ReviewsCreate(
        menu_item_id=menu_item_id,
        customer_name=customer_name,
        rating=rating,
        review_text=review_text
    )
    
    return review_controller.create(db=db, request=request)


@router.get("/view-reviews")
def get_reviews(db: Session = Depends(get_db)):
    """
    Get menu items that have reviews and the reviews
    """
    return customer_services.get_reviews(db=db)


@router.post("/submit-order", response_model=order_submission_schema.OrderSubmissionResponse)
def submit_complete_order(
    order_data: order_submission_schema.CompleteOrderSubmission,
    db: Session = Depends(get_db)
):
    """
    Submit a complete order from the browser with all customer, delivery, and payment information.
    This replaces the need for multiple API calls to build an order.
    """
    return customer_services.submit_complete_order(db=db, order_data=order_data)