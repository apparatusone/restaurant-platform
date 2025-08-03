from sqlalchemy.orm import Session
from sqlalchemy import func
from fastapi import HTTPException, status, Response
from sqlalchemy.exc import SQLAlchemyError
from datetime import datetime, date
from api.models.menu_item_ingredients import MenuItemIngredient
from api.models.resources import Resource
from api.models import promotions as promotion_model
from api.models.payment_method import Payment, PaymentStatus
from api.models.orders import Order, StatusType
from api.models.reviews import Reviews
from api.models.menu_items import MenuItem
from .analytics import ValueSort
    

# this function gets and returns the ingredients needed for a particular menu item
def get_required_ingredients(db, menu_item_id, quantity):
    ingredients = db.query(MenuItemIngredient).join(Resource).filter(
        MenuItemIngredient.menu_item_id == menu_item_id
    ).all()

    return {
        ingredient.resource.item: ingredient.amount * quantity
        for ingredient in ingredients
    }

# staff actions for promotions
def get_promotion_by_code(db: Session, promo_code: str):
    try:
        item = db.query(promotion_model.Promotion).filter(promotion_model.Promotion.code == promo_code).first()
        if not item:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Promo code not found!")
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return item


def update_promotion_by_code(db: Session, promo_code: str, request):
    try:
        item = db.query(promotion_model.Promotion).filter(promotion_model.Promotion.code == promo_code)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Promo code not found!")
        update_data = request.dict(exclude_unset=True)
        item.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return item.first()


def delete_promotion_by_code(db: Session, promo_code: str):
    try:
        item = db.query(promotion_model.Promotion).filter(promotion_model.Promotion.code == promo_code)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Promo code not found!")
        item.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return Response(status_code=status.HTTP_204_NO_CONTENT)


def get_daily_revenue(db: Session, target_date: date):
    """
    Get count of completed orders for a specific date
    Note: Revenue calculation removed as payment_method no longer tracks amount
    """
    # Get a count of the completed orders
    completed_orders = db.query(func.count(Order.id)).join(
        Payment, Order.id == Payment.order_id
    ).filter(
        func.date(Order.order_date) == target_date
    ).filter(
        Payment.status == PaymentStatus.COMPLETED
    ).scalar()
    
    return {
        "date": target_date.isoformat(),
        "completed_orders": completed_orders or 0
    }


def review_feedback(db: Session, rating: int):
    """
    Get reviews filtered by rating
    """
    return db.query(Reviews).join(MenuItem).filter(Reviews.rating == rating).all()


def get_orders_by_status(db: Session, status: StatusType):
    """
    Get orders filtered by status
    """
    return db.query(Order).filter(Order.status == status).all()


def get_orders_by_date_range(db: Session, start_date: date, end_date: date):
    """
    Get orders within a date range
    """
    return db.query(Order).filter(
        func.date(Order.order_date) >= start_date
    ).filter(
        func.date(Order.order_date) <= end_date
    ).all()


def update_order_status(db: Session, order_id: int, status: StatusType):
    """
    Update the status of an order
    """
    from ..controllers import orders as order_controller
    from ..schemas.orders import OrderUpdate
    
    # check if order exists
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")
    
    # update the order status
    try:
        order_update = OrderUpdate(status=status)
        updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
        return updated_order
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update order status: {str(e)}")